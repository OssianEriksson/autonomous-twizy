#ifndef TWIZY_WEBOTS_PIKSI
#define TWIZY_WEBOTS_PIKSI

#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include <ros/ros.h>
#include <string>
#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

class Piksi {
  private:
    std::unique_ptr<webots::GPS> gps_;
    std::unique_ptr<webots::Accelerometer> accelerometer_;
    std::unique_ptr<webots::Gyro> gyro_;

    ros::Publisher pub_imu_;
    ros::Publisher pub_gnss_;
    ros::Timer imu_update_timer_, gnss_update_timer_;

    std::string position_;

    float gnss_cov_, gyro_cov_, accelerometer_cov_;

    geodesy::UTMPoint origin_;

    void imu_update(const ros::TimerEvent &evt);
    void gnss_update(const ros::TimerEvent &evt);

    static geodesy::UTMPoint llh_to_utm_point(double lat, double lon,
                                              double altitude);

  public:
    Piksi(webots::Supervisor &supervisor, ros::NodeHandle &nh,
          ros::NodeHandle &nh_private, std::string position);
    ~Piksi();
};

} // namespace twizy_webots

#endif