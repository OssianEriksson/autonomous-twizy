#include "ackermann_ekf/imu_sensor.h"

#include <geometry_msgs/TransformStamped.h>
#include <string>

namespace ackermann_ekf {

ImuSensor::ImuSensor(SensorArray &sensor_array,
                     const XmlRpc::XmlRpcValue &params, ros::NodeHandle &nh)
    : Sensor(sensor_array, params) {
    // Disallow measurements which cannot be measured by this sensor from beeing
    // fused
    measurement_.mask[Measurement::X] = false;
    measurement_.mask[Measurement::Y] = false;
    measurement_.mask[Measurement::Z] = false;
    measurement_.mask[Measurement::dx_dt] = false;
    measurement_.mask[Measurement::dy_dt] = false;
    measurement_.mask[Measurement::dz_dt] = false;

    accel_mask_ = measurement_.mask[Measurement::d2x_dt2] ||
                  measurement_.mask[Measurement::d2y_dt2] ||
                  measurement_.mask[Measurement::d2z_dt2];
    angular_vel_mask_ = measurement_.mask[Measurement::droll_dt] ||
                        measurement_.mask[Measurement::dpitch_dt] ||
                        measurement_.mask[Measurement::dyaw_dt];
    orientation_mask_ = measurement_.mask[Measurement::Roll] ||
                        measurement_.mask[Measurement::Pitch] ||
                        measurement_.mask[Measurement::Yaw];

    std::string topic = static_cast<std::string>(params["topic"]);

    ROS_INFO("Adding Imu sensor on topic %s", topic.c_str());

    subscriber_ = nh.subscribe(topic, 10, &ImuSensor::callback, this);
}

void ImuSensor::set_cov_xyz(const tf2::Transform transform, const int index[3],
                            const boost::array<double, 9> &cov) {
    tf2::Matrix3x3 covariance_matrix =
        transform.getBasis() *
        tf2::Matrix3x3(cov[0], cov[1], cov[2], cov[3], cov[4], cov[5], cov[6],
                       cov[7], cov[8]) *
        transform.getBasis().transpose();
    for (int i = 0; i < 9; i++) {
        measurement_.R(index[i / 3], index[i % 3]) =
            covariance_matrix[i / 3][i % 3];
    }
}

void ImuSensor::callback(const sensor_msgs::Imu::ConstPtr &msg) {
    // If we have been told to include some piece of data (*_mask_ is true) but
    // the data doesn't exist (*_covariance[0] == -1) we return.
    // \todo Update imu mask depending on whether data is present or not for
    // acceleration, angular velocity and orientation separately instead of
    // togheter
    if ((accel_mask_ && msg->linear_acceleration_covariance[0] == -1) ||
        (angular_vel_mask_ && msg->linear_acceleration_covariance[0] == -1) ||
        (orientation_mask_ && msg->linear_acceleration_covariance[0] == -1)) {
        return;
    }

    geometry_msgs::TransformStamped transform;
    if (!sensor_array_.get_transform(transform, msg->header)) {
        // If no transform exists we cannot determine e.g. sensor position so we
        // are forced to discard the measurement
        return;
    }

    this->set_sensor_position(transform);

    tf2::Transform T;
    tf2::fromMsg(transform.transform, T);
    // Remove translation from transform, we only want rotation
    T.setOrigin(tf2::Vector3(0, 0, 0));

    // Rotate measurements to local coordinate system
    tf2::Quaternion orientation =
        T * tf2::Quaternion(msg->orientation.x, msg->orientation.y,
                            msg->orientation.z, msg->orientation.w);
    tf2::Vector3 angular_velocity =
        T * tf2::Vector3(msg->angular_velocity.x, msg->angular_velocity.y,
                         msg->angular_velocity.z);
    tf2::Vector3 linear_acceleration =
        T * tf2::Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y,
                         msg->linear_acceleration.z);

    tf2::Matrix3x3(orientation)
        .getRPY(measurement_.z(Measurement::Roll),
                measurement_.z(Measurement::Pitch),
                measurement_.z(Measurement::Yaw));
    measurement_.z(Measurement::droll_dt) = angular_velocity.x();
    measurement_.z(Measurement::dpitch_dt) = angular_velocity.y();
    measurement_.z(Measurement::dyaw_dt) = angular_velocity.z();
    measurement_.z(Measurement::d2x_dt2) = linear_acceleration.x();
    measurement_.z(Measurement::d2y_dt2) = linear_acceleration.y();
    measurement_.z(Measurement::d2z_dt2) = linear_acceleration.z();

    int RPY[3] = {Measurement::Roll, Measurement::Pitch, Measurement::Yaw};
    int drpy_dt[3] = {Measurement::droll_dt, Measurement::dpitch_dt,
                      Measurement::dyaw_dt};
    int dxyz2_dt2[3] = {Measurement::d2x_dt2, Measurement::d2y_dt2,
                        Measurement::d2z_dt2};

    set_cov_xyz(T, RPY, msg->orientation_covariance);
    set_cov_xyz(T, drpy_dt, msg->angular_velocity_covariance);
    set_cov_xyz(T, dxyz2_dt2, msg->linear_acceleration_covariance);

    measurement_.time = msg->header.stamp.toSec();

    sensor_array_.process_measurement(measurement_);
}

} // namespace ackermann_ekf