#ifndef ACKERMANN_EKF_SENSOR
#define ACKERMANN_EKF_SENSOR

#include "ackermann_ekf_cpp/ackermann_ekf.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

namespace ackermann_ekf {
class SensorArray;

class Sensor {
  protected:
    SensorArray &sensor_array_;

    Measurement measurement_;

    Sensor(SensorArray &sensor_array, const XmlRpc::XmlRpcValue &params);

    void set_sensor_position(const geometry_msgs::TransformStamped &transform);
};
} // namespace ackermann_ekf

#endif