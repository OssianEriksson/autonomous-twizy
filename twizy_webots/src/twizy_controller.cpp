#include "twizy_webots/twizy_controller.h"
#include "twizy_webots/motors.h"
#include "twizy_webots/piksi.h"
#include "twizy_webots/realsense.h"
#include "twizy_webots/velodyne.h"
#include "twizy_webots/wheel_encoder.h"

#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <rosgraph_msgs/Clock.h>
#include <webots/Node.hpp>

namespace twizy_webots {

TwizyController::TwizyController(ros::NodeHandle &nh,
                                 ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
    std::cout << "Initializing controller\n";

    nh.getParam("/use_sim_time", use_sim_time_);

    clock_publisher_ = nh.advertise<rosgraph_msgs::Clock>("clock", 1);
    position_publisher_ = nh_private.advertise<geometry_msgs::PointStamped>(
        "ground_truth/point", 1);

    supervisor_ = std::unique_ptr<webots::Supervisor>(new webots::Supervisor());
}

void TwizyController::start() {
    new Motors(*supervisor_, nh_);
    new Realsense(*supervisor_, nh_, "front");
    new Realsense(*supervisor_, nh_, "rear");
    new Piksi(*supervisor_, nh_, nh_private_, "left");
    new Piksi(*supervisor_, nh_, nh_private_, "right");
    new WheelEncoder(*supervisor_, nh_, nh_private_, "left");
    new WheelEncoder(*supervisor_, nh_, nh_private_, "right");
    new Velodyne(*supervisor_, nh_);

    webots::Node *robot = supervisor_->getSelf();

    geometry_msgs::PointStamped position;
    position.header.frame_id = "map";
    while (supervisor_->step(supervisor_->getBasicTimeStep()) != -1 &&
           ros::ok()) {
        ros::spinOnce();

        if (use_sim_time_) {
            rosgraph_msgs::Clock clock;
            double time = supervisor_->getTime();
            clock.clock.sec = (int)time;
            // round prevents precision issues that can cause problems with ROS
            // timers
            clock.clock.nsec = round(1000 * (time - clock.clock.sec)) * 1.0e+6;
            clock_publisher_.publish(clock);
        }

        const double *wb_position = robot->getPosition();
        position.header.stamp = ros::Time::now();
        position.point.x = wb_position[2];
        position.point.y = wb_position[0];
        position.point.z = wb_position[1];

        position_publisher_.publish(position);
    }
}

TwizyController::~TwizyController() { supervisor_.reset(); }

} // namespace twizy_webots