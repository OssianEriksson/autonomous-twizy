#ifndef TWIZY_WEBOTS_TWIZY_CONTROLLER
#define TWIZY_WEBOTS_TWIZY_CONTROLLER

#include <memory>
#include <ros/ros.h>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

class TwizyController {
  private:
    bool use_sim_time_ = false;

    ros::Publisher clock_publisher_;
    ros::NodeHandle &nh_, &nh_private_;

    std::unique_ptr<webots::Supervisor> supervisor_;

  public:
    TwizyController(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    ~TwizyController();

    void start();
};

} // namespace twizy_webots

#endif