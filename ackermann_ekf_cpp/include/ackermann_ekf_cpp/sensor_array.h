#ifndef ACKERMANN_EKF_SENSOR_ARRAY
#define ACKERMANN_EKF_SENSOR_ARRAY

#include "ackermann_ekf_cpp/ackermann_ekf.h"
#include "ackermann_ekf_cpp/sensor.h"

#include <boost/ptr_container/ptr_vector.hpp>
#include <iostream>
#include <map>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <Eigen/Dense>

namespace ackermann_ekf {

class SensorArray {
  private:
    AckermannEkf filter_;

    tf2_ros::Buffer &tf_buffer_;

    tf2_ros::TransformListener &tf_listener_;

    Eigen::Vector3d initial_position_;

    bool filter_initialized_ = false;

    float frequency_;

    std::string base_link_;

  public:
    boost::ptr_vector<Sensor> sensors_;

    SensorArray(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

    bool get_transform(geometry_msgs::TransformStamped &transform,
                       const std_msgs::Header &header);
    
    void process_measurement(Measurement &measurement);
};
} // namespace ackermann_ekf

#endif