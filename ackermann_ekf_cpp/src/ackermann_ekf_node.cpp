#include "ackermann_ekf_cpp/sensor_array.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ackermann_ekf");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ackermann_ekf::SensorArray ekf(nh, nh_private);
    ekf.initialize();
    ros::spin();

    return 0;
}
