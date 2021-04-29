#include "twizy_webots/piksi.h"

#include <cmath>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <webots/utils/AnsiCodes.hpp>

namespace twizy_webots {

Piksi::Piksi(webots::Supervisor &supervisor, ros::NodeHandle &nh,
             ros::NodeHandle &nh_private, std::string position)
    : gps_(supervisor.getGPS(position + "_piksi_gnss")),
      accelerometer_(
          supervisor.getAccelerometer(position + "_piksi_accelerometer")),
      gyro_(supervisor.getGyro(position + "_piksi_gyro")),
      origin_(llh_to_utm_point(57.70716, 11.96679, 10.0)),
      position_(position) {

    std::cout << "Initializing " << position << " Piksi GNSS and IMU\n";

    if (!nh_private.getParam("piksi/gyro/cov", gyro_cov_) ||
        !nh_private.getParam("piksi/accelerometer/cov", accelerometer_cov_) ||
        !nh_private.getParam("piksi/gnss/cov", gnss_cov_)) {
        std::cout << webots::AnsiCodes::RED_FOREGROUND
                  << "Required ROS parameters missing"
                  << webots::AnsiCodes::RESET << std::endl;
        return;
    }

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

geodesy::UTMPoint Piksi::llh_to_utm_point(double lat, double lon,
                                          double altitude) {
    geographic_msgs::GeoPoint llh;
    llh.latitude = lat;
    llh.longitude = lon;
    llh.altitude = altitude;
    geodesy::UTMPoint utm(llh);
    return utm;
}

void Piksi::imu_update(const ros::TimerEvent &evt) {
    if (std::isnan(accelerometer_->getValues()[0]) ||
        std::isnan(accelerometer_->getValues()[1]) ||
        std::isnan(accelerometer_->getValues()[2]) ||
        std::isnan(gyro_->getValues()[0]) ||
        std::isnan(gyro_->getValues()[1]) ||
        std::isnan(gyro_->getValues()[2])) {
        return;
    }

    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = position_ + "_piksi_imu";
    imu.orientation_covariance[0] = -1;
    imu.angular_velocity.x = gyro_->getValues()[0];
    imu.angular_velocity.y = gyro_->getValues()[1];
    imu.angular_velocity.z = gyro_->getValues()[2];
    imu.angular_velocity_covariance = {gyro_cov_, 0.0, 0.0, 0.0,      gyro_cov_,
                                       0.0,       0.0, 0.0, gyro_cov_};
    imu.linear_acceleration.x = accelerometer_->getValues()[0];
    imu.linear_acceleration.y = accelerometer_->getValues()[1];
    imu.linear_acceleration.z = accelerometer_->getValues()[2];
    imu.linear_acceleration_covariance = {accelerometer_cov_, 0.0, 0.0, 0.0,
                                          accelerometer_cov_, 0.0, 0.0, 0.0,
                                          accelerometer_cov_};

    pub_imu_.publish(imu);
}

void Piksi::gnss_update(const ros::TimerEvent &evt) {
    if (std::isnan(gps_->getValues()[0]) || std::isnan(gps_->getValues()[1]) ||
        std::isnan(gps_->getValues()[2])) {
        return;
    }

    sensor_msgs::NavSatFix fix;
    fix.header.stamp = ros::Time::now();
    fix.header.frame_id = position_ + "_piksi_gnss";
    fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GALILEO;
    fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    fix.position_covariance = {gnss_cov_, 0.0, 0.0, 0.0,      gnss_cov_,
                               0.0,       0.0, 0.0, gnss_cov_};
    fix.position_covariance_type =
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    if (gps_->getCoordinateSystem() == webots::GPS::WGS84) {
        fix.latitude = gps_->getValues()[0];
        fix.longitude = gps_->getValues()[1];
        fix.altitude = gps_->getValues()[2];
    } else {
        geodesy::UTMPoint utm(gps_->getValues()[2] + origin_.easting,
                              gps_->getValues()[0] + origin_.northing,
                              gps_->getValues()[1] + origin_.altitude,
                              origin_.zone, origin_.band);
        geographic_msgs::GeoPoint geo = geodesy::toMsg(utm);
        fix.latitude = geo.latitude;
        fix.longitude = geo.longitude;
        fix.altitude = geo.altitude;
    }

    pub_gnss_.publish(fix);
}

Piksi::~Piksi() {
    gps_.reset();
    accelerometer_.reset();
    gyro_.reset();
}

} // namespace twizy_webots