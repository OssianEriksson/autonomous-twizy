/**
 * @file piksi.h
 *
 * @brief Simulation of an autonomous Twizy's Piksi Multi kit
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#ifndef TWIZY_WEBOTS_PIKSI
#define TWIZY_WEBOTS_PIKSI

#include <geodesy/utm.h>
#include <geographic_msgs/GeoPoint.h>
#include <ros/ros.h>
#include <string>
#include <webots/Accelerometer.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/Supervisor.hpp>

namespace twizy_webots {

/**
 * @brief Handler class for the simulated Piksi Multi boards and sensors
 */
class Piksi {
  private:
    /**
     * Pointer to instance of a GNSS simulated by webots
     */
    std::unique_ptr<webots::GPS> gps_;

    /**
     * Pointer to instance of an accelorometer simulated by webots, part of the
     * Piksi Multi IMU
     */
    std::unique_ptr<webots::Accelerometer> accelerometer_;

    /**
     * Pointer to instance of a gyro simulated by webots, part of the Piksi
     * Multi IMU
     */
    std::unique_ptr<webots::Gyro> gyro_;

    /**
     * ROS publisher of IMU measurement values
     */
    ros::Publisher pub_imu_;

    /**
     * ROS publisher of GNSS measurement values
     */
    ros::Publisher pub_gnss_;

    /**
     * Timer for publishing of IMU messages
     */
    ros::Timer imu_update_timer_;

    /**
     * Timer for publishing of GNSS messages
     */
    ros::Timer gnss_update_timer_;

    /**
     * String describing this Piksi's position on the physical/model Twizy, e.g.
     * "left" or "right"
     */
    std::string position_;

    /**
     * Covariance of GNSS positioning noise (m^2)
     */
    float gnss_cov_;

    /**
     * Covariance of gyro noise (rad^2/s^2)
     */
    float gyro_cov_;

    /**
     * Covariance of accelerometer noise (m^2/s^4)
     */
    float accelerometer_cov_;

    /**
     * The UTM point on the real earth which corresponds to the origin of world
     * coordinates in Webots
     */
    geodesy::UTMPoint origin_;

    /**
     * Callback for #imu_update_timer_
     */
    void imu_update(const ros::TimerEvent &evt);

    /**
     * Callback for #gnss_update_timer_
     */
    void gnss_update(const ros::TimerEvent &evt);

    /**
     * Convert a latitude, longitude, altitude triplet to UTM
     *
     * @param lat Latitude (degrees)
     * @param lon Longitude (degrees)
     * @param altitude Altitude (meters)
     * @return The UTM point corresponding to lat, lon and altitude
     */
    static geodesy::UTMPoint llh_to_utm_point(double lat, double lon,
                                              double altitude);

  public:
    Piksi(webots::Supervisor &supervisor, ros::NodeHandle &nh,
          ros::NodeHandle &nh_private, std::string position);
    ~Piksi();
};

} // namespace twizy_webots

#endif