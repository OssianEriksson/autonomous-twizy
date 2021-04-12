/**
 * @file navsatfix_sensor.h
 *
 * @brief Contains GNSS sensor for EKF
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef ACKERMANN_EKF_NAVSATFIX_SENSOR
#define ACKERMANN_EKF_NAVSATFIX_SENSOR

#include "ackermann_ekf/sensor.h"
#include "ackermann_ekf/sensor_array.h"

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

namespace ackermann_ekf {

/**
 * @brief GNSS sensor for EKF
 */
class NavSatFixSensor : public Sensor {
  private:
    /**
     * Subscriber to NavSatFix values
     */
    ros::Subscriber subscriber_;

    /**
     * Callback for #subscriber_
     */
    void callback(const sensor_msgs::NavSatFix::ConstPtr &msg);

  public:
    NavSatFixSensor(SensorArray &sensor_array,
                    const XmlRpc::XmlRpcValue &params, ros::NodeHandle &nh);
};

} // namespace ackermann_ekf

#endif