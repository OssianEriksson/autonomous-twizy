/**
 * @file imu_sensor.h
 *
 * @brief Contains inertial measurement unit sensor for EKF
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef ACKERMANN_EKF_IMU_SENSOR
#define ACKERMANN_EKF_IMU_SENSOR

#include "ackermann_ekf/sensor.h"
#include "ackermann_ekf/sensor_array.h"

#include <boost/array.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ackermann_ekf {

/**
 * @brief IMU sensor for EKF
 */
class ImuSensor : public Sensor {
  private:
    /**
     * Wether any measurement component relating to acceleration should be
     * included in filter state correction
     */
    bool accel_mask_;

    /**
     * Wether any measurement component relating to angular velocity should be
     * included in filter state correction
     */
    bool angular_vel_mask_;

    /**
     * Wether any measurement component relating to orientation should be
     * included in filter state correction
     */
    bool orientation_mask_;

    /**
     * Subscriber to IMU values
     */
    ros::Subscriber subscriber_;

    /**
     * Callback for #subscriber_
     */
    void callback(const sensor_msgs::Imu::ConstPtr &msg);

    /**
     * Sets measurement covaraiance values from a 3x3 matrix
     *
     * @param transform A transform to be applied to the covariances
     * @param index Indicies in the measurement vector which are to be populated
     * by covaraiance values
     * @param cov Row major 3x3 matrix of covariance values to set in the
     * measurement covariance matrix
     */
    void set_cov_xyz(const tf2::Transform transform, const int index[3],
                     const boost::array<double, 9> &cov);

  public:
    ImuSensor(SensorArray &sensor_array, const XmlRpc::XmlRpcValue &params,
              ros::NodeHandle &nh);
};

} // namespace ackermann_ekf

#endif