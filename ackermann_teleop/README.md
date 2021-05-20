# ackermann_teleop <!-- omit in toc -->

Teleoperation ("radio control") of Ackermann (car-like) vehicles. To run on a real robot you would for example connect yourself and the robot to the same wifi network, set up [network communication](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) between the two devices and run the nodes provided by this package on your local machine, sending control signals to the robot over wifi.

**Contents**

- [Nodes](#nodes)
  - [key_teleop](#key_teleop)
    - [Published Topics](#published-topics)
    - [Parameters](#parameters)

# Nodes

## key_teleop

Remote control using a keyboard. The controls are very wacky, be ware! (TODO: Make the controls better!) Usage instructions will be printed to the terminal once you start the node (make sure the `output` attribute is set to `screen` if you are using roslaunch).

### Published Topics

* `ackermann_cmd` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/noetic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))  
  Control signal for an Ackermann vehicle.

### Parameters

* `~max_forward_speed` (`double`, required)  
  Maximum forward speed of the vehicle.

* `~max_reverse_speed` (`double`, required)  
  Maximum speed of the vehicle when reversing.

* `~max_steering_angle` (`double`, required)  
  Maximum steering angle of the vehicle.