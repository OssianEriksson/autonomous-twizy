#include "ackermann_ekf_cpp/navsatfix_sensor.h"

namespace ackermann_ekf {
NavSatFixSensor::NavSatFixSensor(const SensorArray &sensor_array,
                                 const XmlRpc::XmlRpcValue &params)
    : Sensor(sensor_array, params), topic_(params["topic"]) {}

void NavSatFixSensor::initialize(const ros::NodeHandle &nh) {
    // measurement_.z_(Measurement::X) = msg->
}
} // namespace ackermann_ekf