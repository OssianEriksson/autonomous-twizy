#include "twizy_webots/motors.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <math.h>
#include <webots/Motor.hpp>

namespace twizy_webots {

Motors::Motors(webots::Supervisor &supervisor, ros::NodeHandle &nh)
    : left_steering_motor_(supervisor.getMotor("front_left_steering_motor")),
      right_steering_motor_(supervisor.getMotor("front_right_steering_motor")),
      left_drive_motor_(supervisor.getMotor("rear_left_wheel_motor")),
      right_drive_motor_(supervisor.getMotor("rear_right_wheel_motor")) {

    ROS_ASSERT(
        nh.getParam("physical/rear_wheel/radius", rear_wheel_radius_) &&
        nh.getParam("physical/max_forward_speed", max_forward_speed_) &&
        nh.getParam("physical/max_reverse_speed", max_reverse_speed_) &&
        nh.getParam("physical/max_steering_angle", max_steering_angle_) &&
        nh.getParam("physical/wheelbase", wheelbase_) &&
        nh.getParam("physical/front_track", front_track_) &&
        nh.getParam("physical/rear_track", rear_track_));

    std::cout << "Initializing motors\n";

    left_drive_motor_->setPosition(std::numeric_limits<double>::infinity());
    right_drive_motor_->setPosition(std::numeric_limits<double>::infinity());
    left_drive_motor_->setVelocity(0.0);
    right_drive_motor_->setVelocity(0.0);

    std::cout << "Subscribing to ackermann_cmd\n";

    subscriber_ = nh.subscribe("ackermann_cmd", 10, &Motors::callback, this);
}

void Motors::callback(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg) {

    double steering_angle = std::max(
        std::min((double)msg->drive.steering_angle, max_steering_angle_),
        -max_steering_angle_);
    double angular_velocity =
        std::max(std::min((double)msg->drive.speed, max_forward_speed_),
                 -max_reverse_speed_) /
        rear_wheel_radius_;

    printf("Speed: %3.3f m/s, Steering angle: %3.3f°\n", msg->drive.speed,
           steering_angle * 180 / M_PI);

    if (abs(steering_angle) > 1e-10) {
        double turning_radius = wheelbase_ / tan(steering_angle);
        left_steering_motor_->setPosition(
            atan(wheelbase_ / (turning_radius - front_track_ / 2.0)));
        right_steering_motor_->setPosition(
            atan(wheelbase_ / (turning_radius + front_track_ / 2.0)));
        left_drive_motor_->setVelocity(angular_velocity *
                                       (turning_radius - rear_track_ / 2.0) /
                                       turning_radius);
        right_drive_motor_->setVelocity(angular_velocity *
                                        (turning_radius + front_track_ / 2.0) /
                                        turning_radius);
    } else {
        left_steering_motor_->setPosition(0.0);
        right_steering_motor_->setPosition(0.0);
        left_drive_motor_->setVelocity(angular_velocity);
        right_drive_motor_->setVelocity(angular_velocity);
    }
}

Motors::~Motors() {
    left_steering_motor_.reset();
    right_steering_motor_.reset();
    left_drive_motor_.reset();
    right_drive_motor_.reset();
}

} // namespace twizy_webots