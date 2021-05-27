/**
 * @file realsense.h
 *
 * @brief Simulation of an autonomous Twizy's Velodyne VLP-16 LiDAR
 *
 * The Velodyne VLP-16 is also called the Velodyne PUCK
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef TWIZY_WEBOTS_VELODYNE
#define TWIZY_WEBOTS_VELODYNE

#include <ros/ros.h>
#include <string>
#include <webots/Lidar.hpp>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

/**
 * @brief Handler class for the simulated Velodyne VLP-16 LiDAR
 */
class Velodyne {
  private:
    /**
     * Pointer to instance of a LiDAR simulated by webots
     */
    std::unique_ptr<webots::Lidar> lidar_;

    /**
     * ROS publisher of the LiDAR point cloud
     */
    ros::Publisher publisher_;

    /**
     * Timer for publising of LiDAR messages
     */
    ros::Timer update_timer_;

    /**
     * Callback for #update_timer_
     */
    void update(const ros::TimerEvent &evt);

  public:
    Velodyne(webots::Supervisor &supervisor, ros::NodeHandle &nh);
    ~Velodyne();
};

} // namespace twizy_webots

#endif