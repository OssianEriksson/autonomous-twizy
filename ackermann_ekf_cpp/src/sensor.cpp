#include "ackermann_ekf_cpp/sensor.h"
#include "ackermann_ekf_cpp/ackermann_ekf.h"

namespace ackermann_ekf
{
    Sensor::Sensor(const SensorArray &sensor_array,
    const  XmlRpc::XmlRpcValue &params)
        : sensor_array_(sensor_array)
    {
        for (int i = 0; i < MEASUREMENT_SIZE; i++)
        {
            fuse_[i] = false;
        }

        XmlRpc::XmlRpcValue fuse = params["fuse"];
        if (fuse.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (int i = 0; i < MEASUREMENT_SIZE; i++)
            {
                fuse_[i] = static_cast<bool>(fuse[i]);
            }
        }
    }
}