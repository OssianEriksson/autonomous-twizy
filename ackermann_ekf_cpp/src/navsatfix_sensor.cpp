#include "ackermann_ekf_cpp/navsatfix_sensor.h"

namespace ackermann_ekf
{
    NavSatFixSensor::NavSatFixSensor(const SensorArray &sensor_array,
                                     std::array<bool, MEASUREMENT_SIZE> fuse)
         : Sensor<sensor_msgs::NavSatFix>(sensor_array, fuse)
    {
    }

    void NavSatFixSensor::callback(const sensor_msgs::NavSatFix &msg)
    {
    }
}