#include "ackermann_ekf_cpp/navsatfix_sensor.h"
#include "ackermann_ekf_cpp/ackermann_ekf.h"
#include "ackermann_ekf_cpp/sensor.h"
#include "ackermann_ekf_cpp/sensor_array.h"

namespace ackermann_ekf {
NavSatFixSensor::NavSatFixSensor(SensorArray &sensor_array,
                                 const XmlRpc::XmlRpcValue &params,
                                 ros::NodeHandle &nh)
    : Sensor(sensor_array, params) {
    measurement_.mask_[Measurement::dx_dt] = false;
    measurement_.mask_[Measurement::dy_dt] = false;
    measurement_.mask_[Measurement::dz_dt] = false;
    measurement_.mask_[Measurement::d2x_dt2] = false;
    measurement_.mask_[Measurement::d2y_dt2] = false;
    measurement_.mask_[Measurement::d2z_dt2] = false;
    measurement_.mask_[Measurement::Roll] = false;
    measurement_.mask_[Measurement::Pitch] = false;
    measurement_.mask_[Measurement::Yaw] = false;
    measurement_.mask_[Measurement::droll_dt] = false;
    measurement_.mask_[Measurement::dpitch_dt] = false;
    measurement_.mask_[Measurement::dyaw_dt] = false;

    std::string topic = static_cast<std::string>(params["topic"]);

    ROS_INFO("Adding NavSatFix sensor on topic %s", topic.c_str());

    subscriber_ = nh.subscribe(topic, 10, &NavSatFixSensor::callback, this);
}

void NavSatFixSensor::callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    ROS_DEBUG("NavSatFix callback called!");

    geometry_msgs::TransformStamped transform;
    if (!sensor_array_.get_transform(transform, msg->header)) {
        return;
    }

    this->set_sensor_position(transform);

    geographic_msgs::GeoPoint llh;
    llh.latitude = msg->latitude;
    llh.longitude = msg->longitude;
    llh.altitude = msg->altitude;
    geodesy::UTMPoint utm(llh);

    measurement_.z_[Measurement::X] = utm.easting;
    measurement_.z_[Measurement::Y] = utm.northing;
    measurement_.z_[Measurement::Z] = utm.altitude;

    const int XYZ[3] = {Measurement::X, Measurement::Y, Measurement::Z};
    for (int i = 0; i < 9; i++) {
        measurement_.R_(XYZ[i / 3], XYZ[i % 3]) = msg->position_covariance[i];
    }

    measurement_.time_ = msg->header.stamp.toSec();

    sensor_array_.process_measurement(measurement_);
}
} // namespace ackermann_ekf