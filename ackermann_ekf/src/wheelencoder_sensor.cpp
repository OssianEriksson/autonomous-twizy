#include "ackermann_ekf/wheelencoder_sensor.h"
#include "ackermann_ekf/ackermann_ekf.h"

#include <geometry_msgs/TransformStamped.h>
#include <string>

namespace ackermann_ekf {

WheelencoderSensor::WheelencoderSensor(SensorArray &sensor_array,
                                       const XmlRpc::XmlRpcValue &params,
                                       ros::NodeHandle &nh)
    : Sensor(sensor_array, params) {
    // Disallow measurements which cannot be measured by this sensor from beeing
    // fused
    for (int i = 0; i < MEASUREMENT_SIZE; i++) {
        // Only the dx_dt measurement is recorded in the WheelEncoder message
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

    // If no transform exists we cannot determine e.g. sensor position so we
    // are forced to discard the measurement
    // Also attempt to bring filter time forward. If this cannot be achived, we
    // must also return. Filter time is brought forward to emulate the Kalman
    // predict step (which will then get skipped once the measurement is
    // processed for real since the filter state is already for the correct
    // time). Since the WheelEncoder message only supports positive speeds (both
    // for forward and backward running) we must predict so we can judge the
    // sign of current velocity according to the filter (see below).
    geometry_msgs::TransformStamped transform;
    if (!sensor_array_.get_transform(transform, msg->header) ||
        !sensor_array_.bring_time_forward_to(measurement_.time)) {
        return;
    }

    this->set_sensor_position(transform);

    // Depending on the sign of forward speed according to filter state, make
    // the measurement reflect that sign since WheelEncoder messages otherwise
    // do not support negative velocities
    double speed = sensor_array_.filter->x(State::speed);
    double dpitch_dx = sensor_array_.filter->x(State::dpitch_dx);
    double dyaw_dx = sensor_array_.filter->x(State::dyaw_dx);
    double y = measurement_.sensor_position(1);
    double z = measurement_.sensor_position(2);
    double h_speed = speed * (dpitch_dx * z - dyaw_dx * y + 1.0);
    measurement_.z(Measurement::dx_dt) = h_speed > 0 ? msg->speed : -msg->speed;
    // This covariance is faked since in reality it also depends e.g. on filter
    // state speed covaraiance (and will in fact not be Gaussian at all...)
    measurement_.R(Measurement::dx_dt, Measurement::dx_dt) = msg->covariance;

    sensor_array_.process_measurement(measurement_);
}

} // namespace ackermann_ekf