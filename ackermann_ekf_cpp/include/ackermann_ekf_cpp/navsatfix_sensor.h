#ifndef ACKERMANN_EKF_NAVSATFIX_SENSOR
#define ACKERMANN_EKF_NAVSATFIX_SENSOR

#include "ackermann_ekf_cpp/sensor.h"
#include "ackermann_ekf_cpp/navsatfix_sensor.h"
#include "ackermann_ekf_cpp/sensor_array.h"

#include <string>

namespace ackermann_ekf
{
    class NavSatFixSensor : public Sensor
    {
    public:
        NavSatFixSensor(const SensorArray &sensor_array,
                        const XmlRpc::XmlRpcValue &params);
    };
}

#endif