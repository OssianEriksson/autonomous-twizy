#include "ackermann_ekf_cpp/sensor.h"

namespace ackermann_ekf {
Sensor::Sensor(const SensorArray &sensor_array,
               const XmlRpc::XmlRpcValue &params)
    : sensor_array_(sensor_array) {
    if (params.hasMember("mask")) {
        XmlRpc::XmlRpcValue mask = params["mask"];
        ROS_ASSERT(mask.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int i = 0; i < MEASUREMENT_SIZE; i++) {
            mask_[i] = static_cast<bool>(mask[i]);
        }
    } else {
        for (int i = 0; i < MEASUREMENT_SIZE; i++) {
            mask_[i] = true;
        }
    }
}
} // namespace ackermann_ekf