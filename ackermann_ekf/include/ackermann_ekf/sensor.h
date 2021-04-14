/**
 * @file sensor.h
 *
 * @brief Contains a class for a sensor whose value is to be integrated into an
 * instance of AckermannEkf
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef ACKERMANN_EKF_SENSOR
#define ACKERMANN_EKF_SENSOR

#include "ackermann_ekf/ackermann_ekf.h"
#include "ackermann_ekf/sensor_array.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>

namespace ackermann_ekf {

// Forward declaration of SensorArray
class SensorArray;

/**
 * @brief Class for a sensor whose value is to be integrated into an instance of
 * AckermannEkf
 */
class Sensor {
  protected:
    /**
     * Reference to the SensorArray holding this sensor
     */
    SensorArray &sensor_array_;

    /**
     * An instance of Measurement to populate and periodically pass to
     * SensorArray::process_measurement
     */
    Measurement measurement_;

    Sensor(SensorArray &sensor_array, const XmlRpc::XmlRpcValue &params);

    /**
     * Sets the Measurement::sensor_position field in #measurement_
     *
     * @param transform Transform between the frame_id of the sensor and the
     * robots base frame
     */
    void set_sensor_position(const geometry_msgs::TransformStamped &transform);
};

} // namespace ackermann_ekf

#endif