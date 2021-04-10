#ifndef ACKERMANN_EKF_SENSOR_ARRAY
#define ACKERMANN_EKF_SENSOR_ARRAY

#include "ackermann_ekf/ackermann_ekf.h"
#include "ackermann_ekf/sensor.h"

#include <Eigen/Dense>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <limits>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace ackermann_ekf {

class SensorArray {
  private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    ros::Publisher odom_publisher_;
    ros::Subscriber control_subscriber_;

    ros::Timer periodic_update_timer_;

    Eigen::Vector3d initial_position_;
    bool filter_initialized_ = false;

    double frequency_ = 30.0;
    std::string world_frame_ = "map", base_link_ = "base_link";
    bool differential_position_ = true;
    double periodic_filter_time_delay_;

    void periodic_update(const ros::TimerEvent &evt);

    void
    control_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr msg);

  public:
    boost::ptr_vector<Sensor> sensor_ptrs;
    std::unique_ptr<AckermannEkf> filter;

    SensorArray(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

    ~SensorArray();

    bool get_transform(geometry_msgs::TransformStamped &transform,
                       const std_msgs::Header &header);

    void process_measurement(Measurement &measurement);
    bool bring_time_forward_to(double time);
};
} // namespace ackermann_ekf

#endif