#include "ackermann_ekf/sensor_array.h"
#include "ackermann_ekf/ackermann_ekf.h"
#include "ackermann_ekf/imu_sensor.h"
#include "ackermann_ekf/navsatfix_sensor.h"
#include "ackermann_ekf/wheelencoder_sensor.h"

#include <geometry_msgs/TransformStamped.h>
#include <limits>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ackermann_ekf {

SensorArray::SensorArray(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
    : tf_buffer_(new tf2_ros::Buffer),
      tf_listener_(new tf2_ros::TransformListener(*tf_buffer_)),
      tf_broadcaster_(new tf2_ros::TransformBroadcaster) {
    ROS_INFO("Initializing Ackermann EKF...");

    nh_private.getParam("base_link", base_link_);
    nh_private.getParam("world_frame", world_frame_);
    nh_private.getParam("frequency", frequency_);
    nh_private.getParam("differential_position", differential_position_);
    nh_private.getParam("zero_z", zero_z_);

    // Default value of periodic_filter_time_delay is two update cycles
    periodic_filter_time_delay_ = 2.0 / frequency_;
    nh_private.getParam("periodic_filter_time_delay",
                        periodic_filter_time_delay_);

    if (!differential_position_) {
        initial_position_.setZero();
        // filter_initialized_ is true as soon as initial_position_ has been set
        filter_initialized_ = true;
    }

    Eigen::VectorXd x_min(STATE_SIZE), x_max(STATE_SIZE);
    x_max.setConstant(std::numeric_limits<double>::max());
    x_min.setConstant(std::numeric_limits<double>::min());
    XmlRpc::XmlRpcValue x_min_value, x_max_value;
    if (nh_private.getParam("x_max", x_max_value)) {
        ROS_ASSERT(x_max_value.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(x_max_value.size() == STATE_SIZE);
        for (int i = 0; i < STATE_SIZE; i++) {
            x_max(i) = static_cast<double>(x_max_value[i]);
        }
    }
    if (nh_private.getParam("x_min", x_min_value)) {
        ROS_ASSERT(x_min_value.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(x_min_value.size() == STATE_SIZE);
        for (int i = 0; i < STATE_SIZE; i++) {
            x_min(i) = static_cast<double>(x_min_value[i]);
        }
    } else {
        // Default x_min value is -x_max
        x_min = -x_max;
    }

    XmlRpc::XmlRpcValue sensors;
    nh_private.getParam("sensors", sensors);
    ROS_ASSERT(sensors.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int i = 0; i < sensors.size(); i++) {
        // For some reason an empty array (I think) is contained in sensors,
        // even when I dont see a reason why. To prevent issues with such
        // things, skip all entries which are not structs/dicts
        if (sensors[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            continue;
        }

        std::string type = sensors[i]["type"];
        if (type == "sensor_msgs/NavSatFix") {
            sensor_ptrs.push_back(new NavSatFixSensor(*this, sensors[i], nh));
        } else if (type == "sensor_msgs/Imu") {
            sensor_ptrs.push_back(new ImuSensor(*this, sensors[i], nh));
        } else if (type == "twizy_wheel_encoder/WheelEncoder") {
            sensor_ptrs.push_back(
                new WheelencoderSensor(*this, sensors[i], nh));
        } else {
            ROS_WARN("Unknown sensor type %s",
                     static_cast<std::string>(type).c_str());
            continue;
        }
    }

    std::string control_topic;
    double wheelbase = 1.0;
    double control_acceleration_gain = 1.0, max_control_acceleration = 1.0;
    double control_angle_speed_gain = 1.0, max_control_angle_speed = 1.0;
    if (nh_private.getParam("control_topic", control_topic)) {
        nh_private.getParam("wheelbase", wheelbase);
        nh_private.getParam("control_acceleration_gain",
                            control_acceleration_gain);
        nh_private.getParam("max_control_acceleration",
                            max_control_acceleration);
        nh_private.getParam("control_angle_speed_gain",
                            control_angle_speed_gain);
        nh_private.getParam("max_control_angle_speed", max_control_angle_speed);
    }

    nh_private.getParam("wheelbase", wheelbase);

    filter = std::unique_ptr<AckermannEkf>(
        new AckermannEkf(x_min, x_max, wheelbase, control_acceleration_gain,
                         max_control_acceleration, control_angle_speed_gain,
                         max_control_angle_speed));
    stable_filter = std::unique_ptr<AckermannEkf>(
        new AckermannEkf(x_min, x_max, wheelbase, control_acceleration_gain,
                         max_control_acceleration, control_angle_speed_gain,
                         max_control_angle_speed));

    if (nh_private.hasParam("control_topic")) {
        ROS_INFO("Subscribing to control signals on topic %s",
                 control_topic.c_str());

        control_subscriber_ = nh.subscribe(
            control_topic, 10, &SensorArray::control_callback, this);
    }

    odom_publisher_ = nh.advertise<nav_msgs::Odometry>("odom", 10);

    // Periodically call back to periodic_update
    periodic_update_timer_ = nh.createTimer(
        ros::Duration(1.0 / frequency_), &SensorArray::periodic_update, this);
}

void SensorArray::control_callback(
    const ackermann_msgs::AckermannDriveStamped::ConstPtr msg) {

    ControlSignal control_signal;
    control_signal.u(ControlSignal::speed) = msg->drive.speed;
    control_signal.u(ControlSignal::angle) = msg->drive.steering_angle;
    control_signal.time = msg->header.stamp.toSec();

    filter->process_control_signal(control_signal);
    stable_filter->process_control_signal(control_signal);
}

void SensorArray::periodic_update(const ros::TimerEvent &evt) {
    // If the callback called this function twise for the same timestamp, i.e.
    // the current time is the same as the last time don't try and republish
    // values for the same timestamp
    if (evt.current_real == evt.last_real) {
        return;
    }

    // Force filter state forward so it doesn't get left behind if there are no
    // measurement values
    this->bring_time_forward_to(evt.current_real.toSec() -
                                periodic_filter_time_delay_);

    tf2::Quaternion q;
    q.setRPY(filter->x(State::Roll), filter->x(State::Pitch),
             filter->x(State::Yaw));

    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = evt.current_real;
    transformStamped.header.frame_id = world_frame_;
    transformStamped.child_frame_id = base_link_;
    transformStamped.transform.translation.x = filter->x(State::X);
    transformStamped.transform.translation.y = filter->x(State::Y);
    transformStamped.transform.translation.z = zero_z_ ? 0.0 : filter->x(State::Z);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transformStamped);

    nav_msgs::Odometry odometry;
    odometry.header.stamp = evt.current_real;
    odometry.header.frame_id = world_frame_;
    odometry.child_frame_id = base_link_;

    odometry.pose.pose.position.x = filter->x(State::X);
    odometry.pose.pose.position.y = filter->x(State::Y);
    odometry.pose.pose.position.z = zero_z_ ? 0.0 : filter->x(State::Z);
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    odometry.pose.pose.orientation.w = q.w();

    const int XYZRPY[6] = {State::X,    State::Y,     State::Z,
                           State::Roll, State::Pitch, State::Yaw};
    for (int i = 0; i < 36; i++) {
        odometry.pose.covariance[i] = filter->P(XYZRPY[i / 6], XYZRPY[i % 6]);
    }

    const double v = filter->x(State::speed), v2 = v * v;
    const double Pv = filter->P(State::speed, State::speed);

    odometry.twist.twist.linear.x = filter->x(State::speed);
    odometry.twist.twist.angular.x = filter->x(State::droll_dx) * v;
    odometry.twist.twist.angular.y = filter->x(State::dpitch_dx) * v;
    odometry.twist.twist.angular.z = filter->x(State::dyaw_dx) * v;

    // Math is hard... This covariance is just guessed and not rigorous in any
    // way whatsoever: We e.g. use only set a diagonal covaraiance matrix
    odometry.twist.covariance[0] = filter->P(State::speed, State::speed);
    odometry.twist.covariance[7] = MIN_COVARIANCE;
    odometry.twist.covariance[14] = MIN_COVARIANCE;
    odometry.twist.covariance[21] =
        filter->P(State::Roll, State::Roll) * v2 +
        Pv * filter->x(State::Roll) * filter->x(State::Roll);
    odometry.twist.covariance[28] =
        filter->P(State::Pitch, State::Pitch) * v2 +
        Pv * filter->x(State::Pitch) * filter->x(State::Pitch);
    odometry.twist.covariance[35] =
        filter->P(State::Yaw, State::Yaw) * v2 +
        Pv * filter->x(State::Yaw) * filter->x(State::Yaw);

    odom_publisher_.publish(odometry);
}

void SensorArray::process_measurement(Measurement &measurement) {
    // If the initial position has not already been determined and the provided
    // measurement contains the neccesary position data set the initial position
    // equal to that
    if (!filter_initialized_ &&
        (measurement.mask[Measurement::X] && measurement.mask[Measurement::Y] &&
         measurement.mask[Measurement::Z])) {
        initial_position_(0) = measurement.z(Measurement::X);
        initial_position_(1) = measurement.z(Measurement::Y);
        initial_position_(2) = measurement.z(Measurement::Z);
        // This subtraction is not neccessary, but is mostly so that the robot
        // will start with height close to zeor (if the robot was upright when
        // the measurement was taken)
        initial_position_ -= measurement.sensor_position;

        // The filter is initialized as soon as initial_position_ has been set
        filter_initialized_ = true;

        // The initial time of the filter is now, when the filtering has been
        // initialized
        filter->time = measurement.time;
        stable_filter->time = measurement.time;
    }

    if (filter_initialized_) {
        // Subtract initial position
        measurement.z(Measurement::X) -= initial_position_(0);
        measurement.z(Measurement::Y) -= initial_position_(1);
        measurement.z(Measurement::Z) -= initial_position_(2);
    } else {
        // Wait unitil filter_initialized_ before continuing
        return;
    }

    filter->process_measurement(measurement);
    stable_filter->process_measurement(measurement);
}

void SensorArray::process_unstable_measurement(Measurement &measurement) {
    if (filter_initialized_) {
        // Subtract initial position
        measurement.z(Measurement::X) -= initial_position_(0);
        measurement.z(Measurement::Y) -= initial_position_(1);
        measurement.z(Measurement::Z) -= initial_position_(2);
    } else {
        // Wait unitil filter_initialized_ before continuing
        return;
    }

    filter->process_measurement(measurement);
}

bool SensorArray::bring_time_forward_to(double time) {
    // There is no point in bringing time forward if the filter has not been
    // initialized and we have no idea what the state should be
    if (filter_initialized_) {
        filter->bring_time_forward_to(time);
        stable_filter->bring_time_forward_to(time);
        return true;
    }
    return false;
}

bool SensorArray::get_transform(geometry_msgs::TransformStamped &transform,
                                const std_msgs::Header &header) {
    try {
        transform = tf_buffer_->lookupTransform(base_link_, header.frame_id,
                                                ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }
    return true;
}

SensorArray::~SensorArray() {
    filter.reset();
    stable_filter.reset();

    tf_buffer_.reset();
    tf_listener_.reset();
    tf_broadcaster_.reset();
}

} // namespace ackermann_ekf
