#ifndef ACKERMANN_EKF_NAVSATFIX_SENSOR
#define ACKERMANN_EKF_NAVSATFIX_SENSOR

#include "ackermann_ekf_cpp/sensor.h"
#include "ackermann_ekf_cpp/sensor_array.h"

#include <sensor_msgs/NavSatFix.h>
#include <array>
#include <string>

namespace ackermann_ekf {
class NavSatFixSensor : public Sensor<sensor_msgs::NavSatFix> {
  public:
    NavSatFixSensor(const SensorArray &sensor_array,
                    std::array<bool, MEASUREMENT_SIZE> fuse);

    void callback(const sensor_msgs::NavSatFix &msg);
};
} // namespace ackermann_ekf

#endif