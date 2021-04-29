#include "twizy_webots/twizy_controller.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "twizy_controller");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    twizy_webots::TwizyController controller(nh, nh_private);
    controller.start();

    return 0;
}
