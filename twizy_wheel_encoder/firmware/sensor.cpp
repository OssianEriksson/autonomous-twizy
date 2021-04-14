#include "sensor.h"
#include "definitions.h"

#include <twizy_wheel_encoder/WheelEncoder.h>

namespace twizy_wheel_encoder {

Sensor::Sensor(const char *topic_name, const Parameters &params,
               const Parameters::Side &side)
    : params_(params), side_(side), publisher(topic_name, &msg_) {
    msg_.speed = 0.0;
}

void Sensor::initialize(twizy_wheel_encoder::NodeHandle &nh, int pin,
                        void (*calls_tick)()) {
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin), calls_tick, RISING);

    nh.advertise(publisher);

    last_time_ = millis();
}

void Sensor::publish(const ros::Time &stamp) {
    unsigned long time = millis();

    msg_.header.frame_id = side_.frame_id;
    msg_.header.stamp = stamp;
    msg_.speed = min(msg_.speed, params_.sensitivity / (time - last_time_));
    msg_.covariance = params_.covariance;
    publisher.publish(&msg_);
}

void Sensor::tick() {
    unsigned long time = millis();
    msg_.speed = params_.sensitivity / (time - last_time_);
    last_time_ = time;
}

} // namespace twizy_wheel_encoder