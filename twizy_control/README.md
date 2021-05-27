# twizy_control <!-- omit in toc -->

Nodes related to control of an autonomous Renault Twizy at the Department of Electrical Engineering, Chalmers University of Technology, Sweden.

This package requires the python package canlib which can be installed with
```bash
pip3 install canlib
```

**Contents**

- [Nodes](#nodes)
  - [can_control](#can_control)
    - [Subscribed Topics](#subscribed-topics)
    - [Parameters](#parameters)
  - [object_avoidance](#object_avoidance)
    - [Subscribed Topics](#subscribed-topics-1)
    - [Published Topics](#published-topics)
  - [twist_to_ackermann](#twist_to_ackermann)
    - [Subscribed Topics](#subscribed-topics-2)
    - [Published Topics](#published-topics-1)
    - [Parameters](#parameters-1)

# Nodes

## can_control

Sends reference steering angle and speed to a real Twizy through a connection to it's CAN bus with a Kvaser device.

### Subscribed Topics

* `ackermann_cmd` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/noetic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))  
  Control signals to send to the Twizy.

* `{~sensors[i]/topic}` (`{~sensors[i]/type}`)
  Sensor data stream to fuse. See the `~sensors` parameter for more information.

### Parameters

* `~timeout` (`int`, default: `150`)  
  CAN connection timeout in milliseconds.

* `~channel` (`int`, default: `0`)  
  Kvaser CAN channel number.

* `~max_steering_angle` (`double`, required)  
  Maximum steering angle of the twizy.

* `~max_forward_speed` (`double`, required)  
  Maximum speed of the Twizy when driving forward.

* `~max_reverse_speed` (`double`, required)  
  Maximum speed of the Twizy when driving in reverse.

## object_avoidance

Node subscribing to depth camera images and bounding boxes corresponding to objects in that depth image and publishes a boolen singaling whether driving in the direction of the camera is allowed or not. As of writing, two kinds of checks are performed reguarding the input data:

1. The published boolean value will signal a full stop as long as any of the following objects are represented in the list of bounding boxes and closer than 3.2 meters from the camera according to the depth image:
   * A person
   * A dog
   * A cat
   * A horse
   After the last reported bounding box of any of these objects is reported the node will wait an additional 2 seconds before giving the signal to drive again.

2. The published boolean value will signal a stop for 10 seconds after the first spotting of a stop sign when there was no stop sign visible before. After the 10 seconds the node will give the signal to drive and ingore all stop signs until all stop signes have gone out of frame, after which the system resets and is able to detect stop signs again.

### Subscribed Topics

* `image` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))  
  Depth image for juding of distances to objects provided on the `boxes` topic. Note that messages on `image` and `boxes` are synchronized by their exact time stamp, so it is important that messages on these topics do not have mismatched timestamps.

* `boxes` ([yolo_detection/BoundingBoxes](../yolo_detection/msg/BoundingBoxes.msg))
  Bounding boxes corresponding to objects in front of the camera. Note that messages on `image` and `boxes` are synchronized by their exact time stamp, so it is important that messages on these topics do not have mismatched timestamps. The following class names are used to represent objects in this node
  | Class name (See Class field of [yolo_detection/BoundingBox](../yolo_detection/msg/BoundingBox.msg)) | Object represented |
  -------------------------------------------------------------------------------------------------------|--------------------|
  | `"person"`                                                                                             | A person           |
  | `"cat"`                                                                                                | A cat              |
  | `"dog"`                                                                                                | A dog              |
  | `"horse"`                                                                                              | A horse            |
  | `"stop sign"`                                                                                          | A stop sign        |

### Published Topics

* `object_in_front` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))  
  Whether or not a vehicle should stop if it's driving in the direction of the camera which provides input values to this node. `true` means the vechicle should stop and `false` means it's supposedly safe to continue driving.

## twist_to_ackermann

Converts [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) to [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/noetic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) as well as incorporating boolean stop commands, one for forward and one for reverse drive. Note that this node expects the z component of angular momentum in the incoming [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) message to be repurposed into meaning absolute steering angle for a car-like vehicle.

### Subscribed Topics

* `cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))  
  Twist messages to be converted into [ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/noetic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html) messages. Note that this node expects the z component of angular momentum in the incoming [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) message to be repurposed into meaning absolute steering angle for a car-like vehicle. 

* `/vision/{front|rear}/object_in_front` ([std_msgs/Bool](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))
  These booleans signal wheter driving in each respective direction is prohibited (represented by a value of `true`) or not (`false`). When e.g. the front boolean is read as `true`, any positive forward speed acoording to the `cmd_vel` topic will be ignored and `0.0` will be published as `speed` on the `ackermann_cmd` topic instead. Negative forward speeds (driving in reverse) will however flow freely from `cmd_vel` to `ackermann_cmd`.

### Published Topics

* `ackermann_cmd` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/noetic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))  
  Message converted from the `cmd_vel` topic taking stop signals from both of `/vision/{front|rear}/object_in_front` into account.

### Parameters

* `~frame_id` (`string`, default: `"odom"`)  
  `frame_id` assigned to messages published on `ackermann_cmd`.
