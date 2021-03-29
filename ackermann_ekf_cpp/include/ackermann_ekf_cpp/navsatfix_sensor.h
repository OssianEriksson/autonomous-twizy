#ifndef ACKERMANN_EKF_NAVSATFIX_SENSOR
#define ACKERMANN_EKF_NAVSATFIX_SENSOR

#include "ackermann_ekf_cpp/sensor.h"
#include "ackermann_ekf_cpp/sensor_array.h"

#include <array>
#include <sensor_msgs/NavSatFix.h>
#include <string>

namespace ackermann_ekf {
class NavSatFixSensor : public Sensor {
  private:
    std::string topic_;

  public:
    NavSatFixSensor(const SensorArray &sensor_array,
                    const XmlRpc::XmlRpcValue &params);

    void initialize(const ros::NodeHandle &nh);
};
} // namespace ackermann_ekf

#endif