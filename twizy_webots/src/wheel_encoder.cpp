#include "twizy_webots/wheel_encoder.h"

#include <cmath>
#include <iostream>
#include <twizy_wheel_encoder/WheelEncoder.h>
#include <webots/utils/AnsiCodes.hpp>

namespace twizy_webots {

WheelEncoder::WheelEncoder(webots::Supervisor &supervisor, ros::NodeHandle &nh,
                           ros::NodeHandle &nh_private, std::string position)
    : sensor_(supervisor.getPositionSensor(position + "_wheel_encoder")),
      position_(position) {

    double ups;
    if (!nh.getParam("/physical/rear_wheel/radius", wheel_radius_) ||
        !nh_private.getParam("wheel_encoder/cov", covariance_) ||
        !nh_private.getParam("wheel_encoder/ups", ups)) {
        std::cout << webots::AnsiCodes::RED_FOREGROUND
                  << "Required ROS parameters missing"
                  << webots::AnsiCodes::RESET << std::endl;
        return;
    }

    std::cout << "Initializing " << position << " wheel encoder\n";

    sensor_->enable(round(1000.0 / ups));

    publisher_ = nh.advertise<twizy_wheel_encoder::WheelEncoder>(
        "/wheel/" + position + "/encoder", 1);

    update_timer_ =
        nh.createTimer(ros::Duration(1.0 / ups), &WheelEncoder::update, this);
}

void WheelEncoder::update(const ros::TimerEvent &evt) {
    double position = sensor_->getValue();
    if (std::isnan(position)) {
        return;
    }

    ros::Time stamp = ros::Time::now();
    double time = stamp.toSec();

    if (time > last_time_ && last_time_ > 0) {
        twizy_wheel_encoder::WheelEncoder enc;
        enc.header.frame_id = "rear_" + position_ + "_wheel";
        enc.header.stamp = stamp;
        enc.speed = abs(wheel_radius_ * (position - last_position_) /
                        (time - last_time_));
        enc.covariance = covariance_;
        publisher_.publish(enc);
    }

    last_position_ = position;
    last_time_ = time;
}

WheelEncoder::~WheelEncoder() { sensor_.reset(); }

} // namespace twizy_webots