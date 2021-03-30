#include "ackermann_ekf_cpp/sensor_array.h"
#include "ackermann_ekf_cpp/ackermann_ekf.h"
#include "ackermann_ekf_cpp/navsatfix_sensor.h"
#include "ackermann_ekf_cpp/sensor.h"

namespace ackermann_ekf {
SensorArray::SensorArray(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : filter_(ros::Time::now().toSec()),
      tf_buffer_(*new tf2_ros::Buffer),
      tf_listener_(*new tf2_ros::TransformListener(tf_buffer_)),
      tf_broadcaster_(*new tf2_ros::TransformBroadcaster),
      base_link_("base_link"),
      world_frame_("map"),
      frequency_(30.0) {
    nh_private.getParam("base_link", base_link_);
    nh_private.getParam("world_frame", world_frame_);
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
            sensor_ptrs.push_back(new NavSatFixSensor(*this, sensors[i], nh));
        } else {
            ROS_WARN("Unknown sensor type %s",
                     static_cast<std::string>(type).c_str());
            continue;
        }
    }

    periodic_update_timer_ = nh.createTimer(
        ros::Duration(1.0 / frequency_), &SensorArray::periodic_update, this);
}

void SensorArray::periodic_update(const ros::TimerEvent &evt) {
    tf2::Quaternion q;
    q.setRPY(filter_.x[State::Roll], filter_.x[State::Pitch],
             filter_.x[State::Yaw]);

    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = world_frame_;
    transformStamped.child_frame_id = base_link_;
    transformStamped.transform.translation.x = filter_.x[State::X];
    transformStamped.transform.translation.y = filter_.x[State::Y];
    transformStamped.transform.translation.z = filter_.x[State::Z];
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(transformStamped);
}

void SensorArray::process_measurement(Measurement &measurement) {
    if (!filter_initialized_ &&
        (measurement.mask[Measurement::X] && measurement.mask[Measurement::Y] &&
         measurement.mask[Measurement::Z])) {
        initial_position_(0) = measurement.z[Measurement::X];
        initial_position_(1) = measurement.z[Measurement::Y];
        initial_position_(2) = measurement.z[Measurement::Z];
        initial_position_ -= measurement.sensor_position;

        filter_.time = measurement.time;

        filter_initialized_ = true;
    }

    if (filter_initialized_) {
        measurement.z[Measurement::X] -= initial_position_(0);
        measurement.z[Measurement::Y] -= initial_position_(1);
        measurement.z[Measurement::Z] -= initial_position_(2);
    } else {
        return;
    }

    double D_Yaw = filter_.x[State::Yaw] - measurement.z[Measurement::Yaw];
    measurement.z[Measurement::Yaw] += round(D_Yaw / (2 * M_PI)) * 2 * M_PI;

    filter_.process_measurement(measurement);
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