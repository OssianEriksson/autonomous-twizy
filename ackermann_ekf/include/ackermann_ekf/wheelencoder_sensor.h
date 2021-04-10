#ifndef ACKERMANN_EKF_WHEELENCODER_SENSOR
#define ACKERMANN_EKF_WHEELENCODER_SENSOR

#include "ackermann_ekf/ackermann_ekf.h"
#include "ackermann_ekf/sensor.h"

#include <ros/ros.h>
#include <string>
#include <geometry_msgs/TransformStamped.h>
#include <twizy_wheel_encoder/WheelEncoder.h>

namespace ackermann_ekf {
class WheelencoderSensor : public Sensor {
  private:
    ros::Subscriber subscriber_;

    void callback(const twizy_wheel_encoder::WheelEncoder::ConstPtr &msg);

  public:
    WheelencoderSensor(SensorArray &sensor_array,
                       const XmlRpc::XmlRpcValue &params, ros::NodeHandle &nh);
};
} // namespace ackermann_ekf

#endif