#ifndef TWIZY_WEBOTS_VELODYNE
#define TWIZY_WEBOTS_VELODYNE

#include <ros/ros.h>
#include <string>
#include <webots/Lidar.hpp>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

class Velodyne {
  private:
    std::unique_ptr<webots::Lidar> lidar_;

    ros::Publisher publisher_;
    ros::Timer update_timer_;

    void update(const ros::TimerEvent &evt);

  public:
    Velodyne(webots::Supervisor &supervisor, ros::NodeHandle &nh);
    ~Velodyne();
};

} // namespace twizy_webots

#endif