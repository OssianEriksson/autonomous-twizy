#include "ackermann_ekf/sensor.h"
#include "ackermann_ekf/ackermann_ekf.h"

namespace ackermann_ekf {

Sensor::Sensor(SensorArray &sensor_array, const XmlRpc::XmlRpcValue &params)
    : sensor_array_(sensor_array) {
    // Parse mask vector
    if (params.hasMember("mask")) {
        XmlRpc::XmlRpcValue mask = params["mask"];
        ROS_ASSERT(mask.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(mask.size() == MEASUREMENT_SIZE);
        for (int i = 0; i < MEASUREMENT_SIZE; i++) {
            measurement_.mask[i] = static_cast<bool>(mask[i]);
        }
    } else {
        // All measurement components are fused into filter. Sensors extending
        // this class are then expexted to remove components from the mask which
        // cannot physically be measured by that sensor
        for (int i = 0; i < MEASUREMENT_SIZE; i++) {
            measurement_.mask[i] = true;
        }
    }

    if (params.hasMember("gravity")) {
        measurement_.gravity = static_cast<double>(params["gravity"]);
    }
}

void Sensor::set_sensor_position(
    const geometry_msgs::TransformStamped &transform) {
    measurement_.sensor_position(0) = transform.transform.translation.x;
    measurement_.sensor_position(1) = transform.transform.translation.y;
    measurement_.sensor_position(2) = transform.transform.translation.z;
}

} // namespace ackermann_ekf