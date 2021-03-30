#include "ackermann_ekf_cpp/sensor_array.h"
#include "ackermann_ekf_cpp/ackermann_ekf.h"
#include "ackermann_ekf_cpp/navsatfix_sensor.h"
#include "ackermann_ekf_cpp/sensor.h"

namespace ackermann_ekf {
SensorArray::SensorArray(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : filter_(ros::Time::now().toSec()),
      tf_buffer_(*new tf2_ros::Buffer),
      tf_listener_(*new tf2_ros::TransformListener(tf_buffer_)),
      base_link_("base_link"),
      frequency_(30.0) {
    nh_private.getParam("base_link", base_link_);
    nh_private.getParam("frequency", frequency_);

    ROS_INFO("Initializing Ackermann EKF...");

    XmlRpc::XmlRpcValue sensors;
    nh_private.getParam("sensors", sensors);
    ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < sensors.size(); i++) {
        if (sensors[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            continue;
        }

        std::string type = sensors[i]["type"];
        if (type == "sensor_msgs/NavSatFix") {
            sensors_.push_back(new NavSatFixSensor(*this, sensors[i], nh));
        } else {
            ROS_WARN("Unknown sensor type %s",
                     static_cast<std::string>(type).c_str());
            continue;
        }
    }
}

void SensorArray::process_measurement(Measurement &measurement) {
    if (!filter_initialized_ && (measurement.mask_[Measurement::X] &&
                                 measurement.mask_[Measurement::Y] &&
                                 measurement.mask_[Measurement::Z])) {
        initial_position_(0) = measurement.z_[Measurement::X];
        initial_position_(1) = measurement.z_[Measurement::Y];
        initial_position_(2) = measurement.z_[Measurement::Z];
        initial_position_ -= measurement.sensor_position_;

        filter_.time_ = measurement.time_;

        filter_initialized_ = true;
    }

    if (filter_initialized_) {
        measurement.z_[Measurement::X] -= initial_position_(0);
        measurement.z_[Measurement::Y] -= initial_position_(1);
        measurement.z_[Measurement::Z] -= initial_position_(2);
    } else {
        return;
    }

    double D_Yaw = filter_.x_[State::Yaw] - measurement.z_[Measurement::Yaw];
    measurement.z_[Measurement::Yaw] += round(D_Yaw / (2 * M_PI)) * 2 * M_PI;

    filter_.process_measurement(measurement);

    std::cout << filter_.x_.transpose() << std::endl;
}

bool SensorArray::get_transform(geometry_msgs::TransformStamped &transform,
                                const std_msgs::Header &header) {
    try {
        transform = tf_buffer_.lookupTransform(base_link_, header.frame_id,
                                               header.stamp);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }
    return true;
}
} // namespace ackermann_ekf