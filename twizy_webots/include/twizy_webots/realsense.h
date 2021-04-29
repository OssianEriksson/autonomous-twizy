#ifndef TWIZY_WEBOTS_REALSENSE
#define TWIZY_WEBOTS_REALSENSE

#include <ros/ros.h>
#include <string>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

class Realsense {
  private:
    std::unique_ptr<webots::Camera> camera_;
    std::unique_ptr<webots::RangeFinder> range_finder_;

    ros::Publisher pub_camera_, pub_camera_info_;
    ros::Publisher pub_range_, pub_range_info_;
    ros::Publisher pub_points_;
    ros::Timer update_timer_;

    std::string position_;

    void update(const ros::TimerEvent &evt);

  public:
    Realsense(webots::Supervisor &supervisor, ros::NodeHandle &nh,
              std::string position);
    ~Realsense();
};

} // namespace twizy_webots

#endif