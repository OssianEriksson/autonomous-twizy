/**
 * @file ackermann_ekf_node.cpp
 *
 * @brief ROS node for sensor fusion using EKF based on Ackermann steering model
 *
 * @author Ossian Eriksson \<ossiane@student.chalmers.se\>
 */

#include "ackermann_ekf/sensor_array.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ackermann_ekf");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ackermann_ekf::SensorArray sensor_array(nh, nh_private);
    ros::spin();

    return 0;
}
