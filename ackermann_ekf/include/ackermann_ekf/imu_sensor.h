#ifndef ACKERMANN_EKF_IMU_SENSOR
#define ACKERMANN_EKF_IMU_SENSOR

#include "ackermann_ekf/ackermann_ekf.h"
#include "ackermann_ekf/sensor.h"

#include <boost/array.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ackermann_ekf {
class ImuSensor : public Sensor {
  private:
    ros::Subscriber subscriber_;

    void callback(const sensor_msgs::Imu::ConstPtr &msg);

    void set_cov_xyz(const tf2::Transform transform, const int index[3],
                     const boost::array<double, 9> &cov);

  public:
    ImuSensor(SensorArray &sensor_array, const XmlRpc::XmlRpcValue &params,
              ros::NodeHandle &nh);
};
} // namespace ackermann_ekf

#endif