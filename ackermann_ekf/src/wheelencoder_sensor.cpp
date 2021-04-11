#include "ackermann_ekf/wheelencoder_sensor.h"
#include "ackermann_ekf/ackermann_ekf.h"
#include "ackermann_ekf/sensor.h"
#include "ackermann_ekf/sensor_array.h"

namespace ackermann_ekf {
WheelencoderSensor::WheelencoderSensor(SensorArray &sensor_array,
                                       const XmlRpc::XmlRpcValue &params,
                                       ros::NodeHandle &nh)
    : Sensor(sensor_array, params) {
    for (int i = 0; i < MEASUREMENT_SIZE; i++) {
        if (i != Measurement::dx_dt) {
            measurement_.mask[i] = false;
        }
    }

    std::string topic = static_cast<std::string>(params["topic"]);

    ROS_INFO("Adding WheelEncoder sensor on topic %s", topic.c_str());

    subscriber_ = nh.subscribe(topic, 10, &WheelencoderSensor::callback, this);
}

void WheelencoderSensor::callback(
    const twizy_wheel_encoder::WheelEncoder::ConstPtr &msg) {
    measurement_.time = msg->header.stamp.toSec();

    geometry_msgs::TransformStamped transform;
    if (!sensor_array_.get_transform(transform, msg->header) ||
        !sensor_array_.bring_time_forward_to(measurement_.time)) {
        return;
    }

    this->set_sensor_position(transform);

    measurement_.z(Measurement::dx_dt) =
        sensor_array_.filter->x(State::speed) > 0 ? msg->speed : -msg->speed;
    measurement_.R(Measurement::dx_dt, Measurement::dx_dt) = msg->covariance;

    sensor_array_.process_measurement(measurement_);
}
} // namespace ackermann_ekf