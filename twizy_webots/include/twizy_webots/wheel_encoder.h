/**
 * @file wheel_encoder.h
 *
 * @brief Simulation of an autonomous Twizy's arduino based wheel encoders
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef TWIZY_WEBOTS_WHEEL_ENCODER
#define TWIZY_WEBOTS_WHEEL_ENCODER

#include <ros/ros.h>
#include <string>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

/**
 * @brief Handler class for the simulated wheel encoders
 */
class WheelEncoder {
  private:
    /**
     * Pointer to instance of a position sensor, mounted on an axis, simulated
     * by webots
     */
    std::unique_ptr<webots::PositionSensor> sensor_;

    /**
     * ROS publisher of wheel encoder messages
     */
    ros::Publisher publisher_;

    /**
     * Timer for publising of camera messages
     */
    ros::Timer update_timer_;

    /**
     * String describing this encoders's position on the physical/model Twizy,
     * e.g. "left" or "right"
     */
    std::string position_;

    /**
     * The radius of the car wheel corresponding to this encoder
     */
    double wheel_radius_;

    /**
     * Covariance of speed measurements published on #publisher_ (m^2/s^2)
     */
    double covariance_;

    /**
     * Timestamp (seconds) of the last time the update() function was called,
     * negative values meaning the function was never called
     */
    double last_time_ = -1;

    /**
     * The position of the position #sensor_ the last time the update() function
     * was called
     */
    double last_position_;

    /**
     * Callback for #update_timer_
     */
    void update(const ros::TimerEvent &evt);

  public:
    WheelEncoder(webots::Supervisor &supervisor, ros::NodeHandle &nh,
                 ros::NodeHandle &nh_private, std::string position);
    ~WheelEncoder();
};

} // namespace twizy_webots

#endif
