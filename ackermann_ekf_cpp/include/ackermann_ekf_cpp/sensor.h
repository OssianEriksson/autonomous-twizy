#ifndef ACKERMANN_EKF_SENSOR
#define ACKERMANN_EKF_SENSOR

#include "ackermann_ekf_cpp/sensor_array.h"
#include "ackermann_ekf_cpp/ackermann_ekf.h"

#include <geometry_msgs/TransformStamped.h>
#include <string>

namespace ackermann_ekf
{
    class Sensor
    {
    protected:
        SensorArray sensor_array_;

        Measurement measurement_;

        bool fuse_[MEASUREMENT_SIZE];

        std::string topic_;

        geometry_msgs::TransformStamped transform_;

        Sensor(const SensorArray &sensor_array, const XmlRpc::XmlRpcValue &params);
    };
}

#endif