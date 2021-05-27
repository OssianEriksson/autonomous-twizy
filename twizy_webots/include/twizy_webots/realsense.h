/**
 * @file realsense.h
 *
 * @brief Simulation of an autonomous Twizy's Intel RealSense cameras
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef TWIZY_WEBOTS_REALSENSE
#define TWIZY_WEBOTS_REALSENSE

#include <ros/ros.h>
#include <string>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

/**
 * @brief Handler class for the simulated Realsense depth and color cameras
 */
class Realsense {
  private:
    /**
     * Pointer to instance of a color camera simulated by webots
     */
    std::unique_ptr<webots::Camera> camera_;

    /**
     * Pointer to instance of a depth camera simulated by webots
     */
    std::unique_ptr<webots::RangeFinder> range_finder_;

    /**
     * ROS publisher of color camera images
     */
    ros::Publisher pub_camera_;

    /**
     * ROS publisher of color camera info
     */
    ros::Publisher pub_camera_info_;

    /**
     * ROS publisher of depth camera images
     */
    ros::Publisher pub_range_;

    /**
     * ROS publisher of depth camera info
     */
    ros::Publisher pub_range_info_;

    /**
     * ROS publisher of a point cloud contstucted from the depth and color
     * camera images
     */
    ros::Publisher pub_points_;

    /**
     * ROS publisher of a cropped version of the point cloud published using
     * #pub_points_
     */
    ros::Publisher pub_cropped_points_;

    /**
     * Timer for publising of camera messages
     */
    ros::Timer update_timer_;

    /**
     * String describing this camera's position on the physical/model Twizy,
     * e.g. "front" or "rear"
     */
    std::string position_;

    /**
     * Callback for #update_timer_
     */
    void update(const ros::TimerEvent &evt);

  public:
    Realsense(webots::Supervisor &supervisor, ros::NodeHandle &nh,
              std::string position);
    ~Realsense();
};

} // namespace twizy_webots

#endif