#ifndef TWIZY_WEBOTS_MOTORS
#define TWIZY_WEBOTS_MOTORS

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <memory>
#include <ros/ros.h>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

class Motors {
  private:
    std::unique_ptr<webots::Motor> left_steering_motor_, right_steering_motor_;
    std::unique_ptr<webots::Motor> left_drive_motor_, right_drive_motor_;

    double rear_wheel_radius_, max_forward_speed_, max_reverse_speed_,
        max_steering_angle_, wheelbase_, front_track_, rear_track_;

    ros::Subscriber subscriber_;

    void callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);

  public:
    Motors(webots::Supervisor &supervisor, ros::NodeHandle &nh);
    ~Motors();
};

} // namespace twizy_webots

#endif