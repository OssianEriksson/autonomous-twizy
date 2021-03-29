#ifndef ACKERMANN_EKF_SENSOR
#define ACKERMANN_EKF_SENSOR

#include "ackermann_ekf_cpp/ackermann_ekf.h"
#include "ackermann_ekf_cpp/sensor_array.h"

#include <ros/ros.h>
#include <array>
#include <geometry_msgs/TransformStamped.h>

namespace ackermann_ekf {
class Sensor {
  protected:
    SensorArray sensor_array_;

    Measurement measurement_;

    std::array<bool, MEASUREMENT_SIZE> mask_;

    std::string topic_;

    geometry_msgs::TransformStamped transform_;

    Sensor(const SensorArray &sensor_array, const XmlRpc::XmlRpcValue &params);

    virtual void initialize(const ros::NodeHandle &nh) = 0;
};
} // namespace ackermann_ekf

#endif