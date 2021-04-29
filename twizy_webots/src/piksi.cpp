#include "twizy_webots/piksi.h"

#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

namespace twizy_webots {

Piksi::Piksi(webots::Supervisor &supervisor, ros::NodeHandle &nh,
             std::string position)
    : gps_(supervisor.getGPS(position + "_piksi_gnss")),
      accelerometer_(
          supervisor.getAccelerometer(position + "_piksi_accelerometer")),
      gyro_(supervisor.getGyro(position + "_piksi_gyro")),
      position_(position) {

    std::cout << "Initializing " << position << "Piksi GNSS and IMU"
              << std::endl;

    gnss_noise_ = supervisor.getFromDevice(gps_)->getProtoField("accuracy")->getSFFloat();
    gnss_noise_ = gnss_noise_ * gnss_noise_;

    double imu_ups = 100.0;
    double gnss_ups = 10.0;

    gps_->enable(round(1000.0 / gnss_ups));
    accelerometer_->enable(round(1000.0 / imu_ups));
    gyro_->enable(round(1000.0 / imu_ups));

    pub_imu_ =
        nh.advertise<sensor_msgs::Imu>("/piksi/" + position + "/imu/raw", 1);
    pub_gnss_ = nh.advertise<sensor_msgs::NavSatFix>(
        "/piksi/" + position + "/gnss/fix", 1);

    imu_update_timer_ =
        nh.createTimer(ros::Duration(1.0 / imu_ups), &Piksi::imu_update, this);
    gnss_update_timer_ = nh.createTimer(ros::Duration(1.0 / gnss_ups),
                                        &Piksi::gnss_update, this);
}

void Piksi::imu_update(const ros::TimerEvent &evt) {}

void Piksi::gnss_update(const ros::TimerEvent &evt) {
    ros::Time stamp = ros::Time::now();

    sensor_msgs::NavSatFix fix;
    fix.position_covariance_type =
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GALILEO;
    fix.header.stamp = stamp;
    fix.header.frame_id = position_ + "_piksi_imu";
    if (gps_->getCoordinateSystem() == webots::GPS::WGS84) {
        fix.latitude = gps_->getValues()[0];
        fix.longitude = gps_->getValues()[1];
        fix.altitude = gps_->getValues()[2];
    } else {
        // geometry_msgs::PointStamped value;
        // value.header.stamp = ros::Time::now();
        // value.header.frame_id =
        //     mRos->name() + '/' + RosDevice::fixedDeviceName();
        // value.point.x = mGPS->getValues()[0];
        // value.point.y = mGPS->getValues()[1];
        // value.point.z = mGPS->getValues()[2];
        // publisher.publish(value);
    }

    publisher.publish(fix);
}

Piksi::~Piksi() {
    gnss_.reset();
    accelerometer_.reset();
    gyro_.reset();
}

} // namespace twizy_webots