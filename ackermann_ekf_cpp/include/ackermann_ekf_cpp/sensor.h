#ifndef ACKERMANN_EKF_SENSOR
#define ACKERMANN_EKF_SENSOR

#include "ackermann_ekf_cpp/ackermann_ekf.h"
#include "ackermann_ekf_cpp/sensor_array.h"

#include <array>
#include <geometry_msgs/TransformStamped.h>

namespace ackermann_ekf {
template <class RosMessage> class Sensor {
protected:
  SensorArray sensor_array_;

  Measurement measurement_;

  std::array<bool, MEASUREMENT_SIZE> fuse_;

  std::string topic_;

  geometry_msgs::TransformStamped transform_;

  Sensor(const SensorArray &sensor_array,
         std::array<bool, MEASUREMENT_SIZE> fuse)
      : sensor_array_(sensor_array), fuse_(fuse) {}

  virtual void callback(const RosMessage &msg) = 0;
};
} // namespace ackermann_ekf

#endif