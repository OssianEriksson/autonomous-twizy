#ifndef ACKERMANN_EKF_SENSOR
#define ACKERMANN_EKF_SENSOR

#include "ackermann_ekf_cpp/sensor_array.h"
#include "ackermann_ekf_cpp/ackermann_ekf.h"

#include <geometry_msgs/TransformStamped.h>
#include <array>

namespace ackermann_ekf
{
    class Sensor
    {
    protected:
        SensorArray sensor_array_;

        Measurement measurement_;

        std::array<bool, MEASUREMENT_SIZE> fuse_;

        std::string topic_;

        geometry_msgs::TransformStamped transform_;

        Sensor(const SensorArray &sensor_array, std::array<bool, MEASUREMENT_SIZE> fuse);
    };
}

#endif