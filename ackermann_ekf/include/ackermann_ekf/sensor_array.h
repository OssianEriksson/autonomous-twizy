/**
 * @file sensor_array.h
 *
 * @brief Sensor fusion using extended Kalman filter
 *
 * This class oversees all the sensors to be fused and also holds the reference
 * to the EKF instance. All measurements taken by sensors pass through the
 * SensorArray class defined here class before entering the EKF.
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef ACKERMANN_EKF_SENSOR_ARRAY
#define ACKERMANN_EKF_SENSOR_ARRAY

#include "ackermann_ekf/ackermann_ekf.h"
#include "ackermann_ekf/sensor.h"

#include <Eigen/Dense>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace ackermann_ekf {

// Forward declare Sensor
class Sensor;

/**
 * @brief Hub of Sensor instances, performs sensor fusion
 *
 * Creates and holds referenses to all sensors whose data is to be integrated
 * into the EKF instance also held by this class. All measurement values
 * recorded by sensors are passed through the process_measurement() function
 * where they are enhanced, e.g. by subtracting an initial position for the
 * "differential mode"
 */
class SensorArray {
  private:
    /**
     * Transform buffer buffers old transforms so they are available for a while
     * after they were first published
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    /**
     * Instance of transform listener for getting tf transforms
     */
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    /**
     * Instance of transform broadcaster e.g. for broadcasting tf transforms
     * based on filter state
     */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /**
     * Publisher of odometry messages
     */
    ros::Publisher odom_publisher_;

    /**
     * Subscriber to control signals which are then used in the EKF predict step
     */
    ros::Subscriber control_subscriber_;

    /**
     * Timer for periodically updating filter state and publishing transforms
     * and messages
     */
    ros::Timer periodic_update_timer_;

    /**
     * Initial (according to the first recorded measurement of position)
     * position of the vehicle
     */
    Eigen::Vector3d initial_position_;

    /**
     * Whether the #filter is ready to recieve measuremenets
     */
    bool filter_initialized_ = false;

    /**
     * Frequency to force update state and publish transforms and messages at
     */
    double frequency_ = 30.0;

    /**
     * tf frame id of the world frame
     */
    std::string world_frame_ = "map";

    /**
     * tf frame id of the frame corresponding to the base of the vehicle
     */
    std::string base_link_ = "base_link";

    /**
     * Wheter to subtract #initial_position_ from position measurements so the
     * vehicle will initially be located close to the origin in the
     * #world_frame_
     */
    bool differential_position_ = true;

    /**
     * If no measurement has been seen for a while the EKF state is forcefully
     * brought forward in time using its predict step. It is then brought
     * forward to the time given by current_time - periodic_filter_time_delay_
     */
    double periodic_filter_time_delay_;

    /**
     * Callback for #periodic_update_timer_
     */
    void periodic_update(const ros::TimerEvent &evt);

    /**
     * Callback for #control_subscriber_
     */
    void
    control_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr msg);

  public:
    /**
     * Holds pointers to all sensors so they will not go out of scope and be
     * deleted
     */
    boost::ptr_vector<Sensor> sensor_ptrs;

    /**
     * Instance of Ackermann EKF filter
     */
    std::unique_ptr<AckermannEkf> filter;

    SensorArray(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

    ~SensorArray();

    /**
     * @param transform Value to be populated by a transform from
     * header.frame_id to #base_link_
     * @param header Header containing a frame_id field
     * @return true if the operation was performed successfully, false otherwise
     */
    bool get_transform(geometry_msgs::TransformStamped &transform,
                       const std_msgs::Header &header);

    /**
     * Process a measurement from a sensor and fuse it into the filter state
     */
    void process_measurement(Measurement &measurement);

    /**
     * Brings the filter state forward to the provided time using the Kalman
     * predict step
     */
    bool bring_time_forward_to(double time);
};

} // namespace ackermann_ekf

#endif