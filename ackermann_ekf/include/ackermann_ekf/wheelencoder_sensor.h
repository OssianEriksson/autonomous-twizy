/**
 * @file wheelencoder_sensor.h
 *
 * @brief Contains wheel encoder sensor for use with an EKF
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef ACKERMANN_EKF_WHEELENCODER_SENSOR
#define ACKERMANN_EKF_WHEELENCODER_SENSOR

#include "ackermann_ekf/sensor.h"

#include <ros/ros.h>
#include <twizy_wheel_encoder/WheelEncoder.h>

namespace ackermann_ekf {

/**
 * @brief Wheel encoder sensor for EKF
 */
class WheelencoderSensor : public Sensor {
  private:
    /**
     * Subscriber to wheel encoder values
     */
    ros::Subscriber subscriber_;

    /**
     * Callback for #subscriber_
     */
    void callback(const twizy_wheel_encoder::WheelEncoder::ConstPtr &msg);

  public:
    WheelencoderSensor(SensorArray &sensor_array,
                       const XmlRpc::XmlRpcValue &params, ros::NodeHandle &nh);
};

} // namespace ackermann_ekf

#endif