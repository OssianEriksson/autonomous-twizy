#include "twizy_webots/twizy_controller.h"
#include "twizy_webots/motors.h"
#include "twizy_webots/realsense.h"
#include "twizy_webots/piksi.h"

#include <rosgraph_msgs/Clock.h>
#include <iostream>

namespace twizy_webots {

TwizyController::TwizyController(ros::NodeHandle &nh,
                                 ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh) {
    std::cout << "Initializing controller\n";

    clock_publisher_ = nh.advertise<rosgraph_msgs::Clock>("clock", 1);
    supervisor_ = std::unique_ptr<webots::Supervisor>(new webots::Supervisor());
    nh.getParam("/use_sim_time", use_sim_time_);
}

void TwizyController::start() {
    new Motors(*supervisor_, nh_);
    new Realsense(*supervisor_, nh_, "front");
    new Realsense(*supervisor_, nh_, "rear");
    new Piksi(*supervisor_, nh_, "left");

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
    }
}

TwizyController::~TwizyController() { supervisor_.reset(); }

} // namespace twizy_webots