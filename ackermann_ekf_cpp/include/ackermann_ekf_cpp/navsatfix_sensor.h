#ifndef ACKERMANN_EKF_NAVSATFIX_SENSOR
#define ACKERMANN_EKF_NAVSATFIX_SENSOR

#include "ackermann_ekf_cpp/sensor.h"
#include "ackermann_ekf_cpp/navsatfix_sensor.h"
#include "ackermann_ekf_cpp/sensor_array.h"

#include <string>
#include <array>

namespace ackermann_ekf
{
    class NavSatFixSensor : public Sensor
    {
    public:
        NavSatFixSensor(const SensorArray &sensor_array, std::array<bool, MEASUREMENT_SIZE> fuse);
    };
}

#endif