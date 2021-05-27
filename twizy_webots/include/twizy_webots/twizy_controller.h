/**
 * @file twizy_controller.h
 *
 * @brief Control of a simulated autonomous Twizy
 *
 * This file contains definitions for the "brain" of the Twizy Webots controller
 * which handles connections to simulated sensors and overarching processing
 * like publishing ROS clock messages
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef TWIZY_WEBOTS_TWIZY_CONTROLLER
#define TWIZY_WEBOTS_TWIZY_CONTROLLER

#include <memory>
#include <ros/ros.h>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

/**
 * @brief "Brain" of the Twizy Webots controller
 *
 * This class should initialize sensors, make sure sensors publish their
 * readings, as well as handle overarching processing like publishing ROS clock
 * messages
 */
class TwizyController {
  private:
    /**
     * Whether ROS uses simulated time as controlled by the ROS parameter
     * /use_sim_time
     */
    bool use_sim_time_ = false;

    /**
     * ROS publisher of clock messages
     */
    ros::Publisher clock_publisher_;

    /**
     * ROS publisher of the simulated Twizy's ground truth position
     */
    ros::Publisher position_publisher_;

    /**
     * Reference to the ROS public node handle
     */
    ros::NodeHandle &nh_;

    /**
     * Reference to the ROS public node handle
     */
    ros::NodeHandle &nh_private_;

    /**
     * Pointer to the Webots supervisor for the Twizy robot
     */
    std::unique_ptr<webots::Supervisor> supervisor_;

  public:
    TwizyController(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~TwizyController();

    /**
     * Starts the Twizy Webots controller
     */
    void start();
};

} // namespace twizy_webots

#endif