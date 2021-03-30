#include "ackermann_ekf_cpp/sensor.h"
#include "ackermann_ekf_cpp/ackermann_ekf.h"
#include "ackermann_ekf_cpp/navsatfix_sensor.h"
#include "ackermann_ekf_cpp/sensor_array.h"

namespace ackermann_ekf {
Sensor::Sensor(SensorArray &sensor_array, const XmlRpc::XmlRpcValue &params)
    : sensor_array_(sensor_array) {
    if (params.hasMember("mask")) {
        XmlRpc::XmlRpcValue mask = params["mask"];
        ROS_ASSERT(mask.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int i = 0; i < MEASUREMENT_SIZE; i++) {
            measurement_.mask[i] = static_cast<bool>(mask[i]);
        }
    } else {
        for (int i = 0; i < MEASUREMENT_SIZE; i++) {
            measurement_.mask[i] = true;
        }
    }
}

void Sensor::set_sensor_position(
    const geometry_msgs::TransformStamped &transform) {
    measurement_.sensor_position(0) = transform.transform.translation.x;
    measurement_.sensor_position(1) = transform.transform.translation.y;
    measurement_.sensor_position(2) = transform.transform.translation.z;
}

} // namespace ackermann_ekf