#ifndef ACKERMANN_EKF_SENSOR_ARRAY
#define ACKERMANN_EKF_SENSOR_ARRAY

#include "ackermann_ekf_cpp/ackermann_ekf.h"
#include "ackermann_ekf_cpp/sensor.h"

#include <Eigen/Dense>
#include <boost/ptr_container/ptr_vector.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace ackermann_ekf {

class SensorArray {
  private:
    AckermannEkf filter_;

    tf2_ros::Buffer &tf_buffer_;

    tf2_ros::TransformListener &tf_listener_;

    tf2_ros::TransformBroadcaster &tf_broadcaster_;

    ros::Timer periodic_update_timer_;

    Eigen::Vector3d initial_position_;

    bool filter_initialized_ = false;

    float frequency_;

    std::string world_frame_;

    std::string base_link_;

    void periodic_update(const ros::TimerEvent &evt);

  public:
    boost::ptr_vector<Sensor> sensor_ptrs;

    SensorArray(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

    bool get_transform(geometry_msgs::TransformStamped &transform,
                       const std_msgs::Header &header);

    void process_measurement(Measurement &measurement);
};
} // namespace ackermann_ekf

#endif