#include "ackermann_ekf_cpp/navsatfix_sensor.h"
#include "ackermann_ekf_cpp/sensor_array.h"

namespace ackermann_ekf
{
    NavSatFixSensor::NavSatFixSensor(const SensorArray &sensor_array,
                                     std::array<bool, MEASUREMENT_SIZE> fuse)
        : Sensor(sensor_array, fuse)
    {
    }
}