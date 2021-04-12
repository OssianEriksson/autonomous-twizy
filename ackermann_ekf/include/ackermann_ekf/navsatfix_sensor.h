#ifndef ACKERMANN_EKF_NAVSATFIX_SENSOR
#define ACKERMANN_EKF_NAVSATFIX_SENSOR

#include "ackermann_ekf/sensor.h"
#include "ackermann_ekf/sensor_array.h"

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

namespace ackermann_ekf {

class NavSatFixSensor : public Sensor {
  private:
    ros::Subscriber subscriber_;

    void callback(const sensor_msgs::NavSatFix::ConstPtr &msg);

  public:
    NavSatFixSensor(SensorArray &sensor_array,
                    const XmlRpc::XmlRpcValue &params, ros::NodeHandle &nh);
};

} // namespace ackermann_ekf

#endif