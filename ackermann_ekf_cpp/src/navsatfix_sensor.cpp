#include "ackermann_ekf_cpp/navsatfix_sensor.h"
#include "ackermann_ekf_cpp/sensor_array.h"

namespace ackermann_ekf
{
    NavSatFixSensor::NavSatFixSensor(const SensorArray &sensor_array,
                                     const XmlRpc::XmlRpcValue &params)
        : Sensor(sensor_array, params)
    {
    }
}