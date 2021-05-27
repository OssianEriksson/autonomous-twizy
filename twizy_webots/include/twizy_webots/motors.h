/**
 * @file motors.h
 *
 * @brief Control of a simulated autonomous Twizy's motors
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef TWIZY_WEBOTS_MOTORS
#define TWIZY_WEBOTS_MOTORS

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <memory>
#include <ros/ros.h>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

/**
 * @brief Handler class for all of the Twizy's motors
 */
class Motors {
  private:
    /**
     * Pointer to instance of the webots motor controlling the steering angle of
     * the front left wheel of the Twizy
     */
    std::unique_ptr<webots::Motor> left_steering_motor_;

    /**
     * Pointer to instance of the webots motor controlling the steering angle of
     * the front right wheel of the Twizy
     */
    std::unique_ptr<webots::Motor> right_steering_motor_;

    /**
     * Pointer to instance of the webots motor controlling the rotation of the
     * rear left wheel of the Twizy
     */
    std::unique_ptr<webots::Motor> left_drive_motor_;

    /**
     * Pointer to instance of the webots motor controlling the rotation of the
     * rear right wheel of the Twizy
     */
    std::unique_ptr<webots::Motor> right_drive_motor_;

    /**
     * Radius of rear wheels
     */
    double rear_wheel_radius_;

    /**
     * Maximum allowed forward speed of the Twizy
     */
    double max_forward_speed_;

    /**
     * Maximum allowed backward speed of the Twizy
     */
    double max_reverse_speed_;

    /**
     * Maximum steering angle allowed (at the centerline of the vehicle if using
     * Ackermann steering)
     */
    double max_steering_angle_;

    /**
     * Distance between the Twizy's front and rear axle
     */
    double wheelbase_;

    /**
     * Distance between centers of the Twizy's front wheels
     */
    double front_track_;

    /**
     * Distance between centers of the Twizy's rear wheels
     */
    double rear_track_;

    /**
     * Subscriber to motor control signals
     */
    ros::Subscriber subscriber_;

    /**
     * Callback for #subscriber_
     */
    void callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &msg);

  public:
    Motors(webots::Supervisor &supervisor, ros::NodeHandle &nh);
    ~Motors();
};

} // namespace twizy_webots

#endif