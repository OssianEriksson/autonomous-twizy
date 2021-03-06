#include "ackermann_ekf/navsatfix_sensor.h"

#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

namespace ackermann_ekf {

NavSatFixSensor::NavSatFixSensor(SensorArray &sensor_array,
                                 const XmlRpc::XmlRpcValue &params,
                                 ros::NodeHandle &nh)
    : Sensor(sensor_array, params) {
    // Disallow measurements which cannot be measured by this sensor from beeing
    // fused
    measurement_.mask[Measurement::dx_dt] = false;
    measurement_.mask[Measurement::dy_dt] = false;
    measurement_.mask[Measurement::dz_dt] = false;
    measurement_.mask[Measurement::d2x_dt2] = false;
    measurement_.mask[Measurement::d2y_dt2] = false;
    measurement_.mask[Measurement::d2z_dt2] = false;
    measurement_.mask[Measurement::Roll] = false;
    measurement_.mask[Measurement::Pitch] = false;
    measurement_.mask[Measurement::Yaw] = false;
    measurement_.mask[Measurement::droll_dt] = false;
    measurement_.mask[Measurement::dpitch_dt] = false;
    measurement_.mask[Measurement::dyaw_dt] = false;

    std::string topic = static_cast<std::string>(params["topic"]);

    ROS_INFO("Adding NavSatFix sensor on topic %s", topic.c_str());

    subscriber_ = nh.subscribe(topic, 10, &NavSatFixSensor::callback, this);
}

void NavSatFixSensor::callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    geometry_msgs::TransformStamped transform;
    if (!sensor_array_.get_transform(transform, msg->header)) {
        // If no transform exists we cannot determine e.g. sensor position so we
        // are forced to discard the measurement
        return;
    }

    this->set_sensor_position(transform);

    // Convert from latitude, longitude to UTM (Cartesian coordinates)
    geographic_msgs::GeoPoint llh;
    llh.latitude = msg->latitude;
    llh.longitude = msg->longitude;
    llh.altitude = msg->altitude;
    geodesy::UTMPoint utm(llh);

    measurement_.z(Measurement::X) = utm.easting;
    measurement_.z(Measurement::Y) = utm.northing;
    measurement_.z(Measurement::Z) = utm.altitude;

    const int XYZ[3] = {Measurement::X, Measurement::Y, Measurement::Z};
    for (int i = 0; i < 9; i++) {
        measurement_.R(XYZ[i / 3], XYZ[i % 3]) = msg->position_covariance[i];
    }

    measurement_.time = msg->header.stamp.toSec();

    sensor_array_.process_measurement(measurement_);
}

} // namespace ackermann_ekf