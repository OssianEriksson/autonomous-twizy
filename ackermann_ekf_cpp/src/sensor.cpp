#include "ackermann_ekf_cpp/sensor.h"

namespace ackermann_ekf
{
    Sensor::Sensor(const SensorArray &sensor_array, std::array<bool, MEASUREMENT_SIZE> fuse)
        : sensor_array_(sensor_array),
        fuse_(fuse)
    {

    }
}