#ifndef TWIZY_WEBOTS_WHEEL_ENCODER
#define TWIZY_WEBOTS_WHEEL_ENCODER

#include <ros/ros.h>
#include <string>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

class WheelEncoder {
  private:
    std::unique_ptr<webots::PositionSensor> sensor_;

    ros::Publisher publisher_;
    ros::Timer update_timer_;

    std::string position_;
    double wheel_radius_;
    double covariance_;
    double last_time_ = -1;
    double last_position_;

    void update(const ros::TimerEvent &evt);

  public:
    WheelEncoder(webots::Supervisor &supervisor, ros::NodeHandle &nh,
                 ros::NodeHandle &nh_private, std::string position);
    ~WheelEncoder();
};

} // namespace twizy_webots

#endif