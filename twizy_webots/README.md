# twizy_webots <!-- omit in toc -->

Simulation of an autonomous Renault Twizy in the Webots real time robotics simulator.

**Contents**

- [Twizy Capabilities](#twizy-capabilities)
- [Running an Existing World](#running-an-existing-world)
- [Creating a new Webots World](#creating-a-new-webots-world)
- [Twizy Webots Controller](#twizy-webots-controller)
  - [Subscribed Topics](#subscribed-topics)
  - [Published Topics](#published-topics)
    - [Parameters](#parameters)

# Twizy Capabilities

The simulated capabilities of the Twizy include
* Forward and reverse driving, steering, emulating nodes from [twizy_control](../twizy_control)
* Subset of the Piksi Multi kit functionality from [piksi_multi_rtk](../piksi_multi_rtk)
* Subset of Velodyne VLP-16 (PUCK) functionality from [velodyne_pointcloud](https://wiki.ros.org/velodyne_pointcloud?distro=noetic)
* Subset of Intel RealSense D435 functionality from [realsense2_camera](https://github.com/IntelRealSense/realsense-ros)
* Wheel encoder functionality from [twizy_wheel_encoder](../twizy_wheel_encoder)

Also compare topic names with repams present in the launch files of [twizy_bringup](../twizy_bringup), they should be identical to enable drop in simulation.

# Running an Existing World

This package does not directly include any ROS node, but does however include a Webots controller which is a ROS node and which publishes topics. This node/controller is started by webots and not e.g. by roslaunch directly. Descriptions of the Twizy robot also need to be generated dynamically before Webots starts.

The launch file [webots.launch](launch/webots.launch) takes care of this for you: This launch file generates a PROTO representation of the Twizy based on the [twizy_description](../twizy_description#readme) package, makes it available to Webots which is started last. Webots then (assuming the Twizy model is part of the provided world file) starts the Twizy Webots Controller which publishes and subscribes to ROS topics. Run this launch file e.g. like this:
```bash
roslaunch twizy_webots webots.launch world:=flat.wbt
```

# Creating a new Webots World

This section describes creating a new simulating world containging the Twizy (I apologize for unclear instructions, I'm writing these from memory...)

1. Make sure you have ran an already existing world (see [Running an Existing World](#running-an-existing-world)) at least once beforehand as this will generate the required Twizy PROTO for you and install it in the correct location
   
2. Open Webots in any way you like
   
3. Select the new world wizard
   
4. Pause the simulation and save the world file inside the [worlds/](worlds) directory
   
5. Open the world file in a text editor and make sure the file contains a `WorldInfo` node with at least the following fields defined:
   ```
   WorldInfo {
     coordinateSystem "NUE"
     contactProperties [
       ContactProperties {
         softCFM 0.0001
       }
     ]
   }
   ```
   This is to discorage the Twizy from sinking a bit too deep into the ground. (The car will sill sink a bit into the ground but not as much)

6. Reload the world in Webots, make sure the simulation does not run or pause it or rewind it back to time 0
   
7. Find the menu for adding a new node/object to the scene, select protos in the top most tree selector and find the Twizy PROTO. Add it to the scene

8. Save the world, make sure the simulation has not ran and then close Webots

You should now hopefully be able to start the simulation using
```bash
roslaunch twizy_webots webots.launch world:=<your_world_file_name_here>.wbt
```

# Twizy Webots Controller

The Twizy webots controller is a ROS node in a program controlled by Webots. See [Running an Existing World](#running-an-existing-world) for more info.

## Subscribed Topics

**Motor Control**

* `ackermann_cmd` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/en/noetic/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))  
  Reference speed and steering angle to control the simulated car.

## Published Topics

* `clock` ([rosgraph_msgs/Clock](http://docs.ros.org/en/noetic/api/rosgraph_msgs/html/msg/Clock.html))  
  Time stamp in the Webots simulation, only published if the `/use_sim_time` parameter is set to `true`.

* `~ground_truth/point` ([geometry_msgs/PointStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html))  
  Ground truth Twizy position in the simulation.

**Piksi Multi**

* `/piksi/{left|right}/imu/raw` ([sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)  
  Measurement values from the IMUs present on the Piksi Multi boards.

* `/piksi/{left|right}/gnss/fix` ([sensor_msgs/NavSatFix](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html)  
  Measurement values from from the Piksi Multi RTK GNSS.

**Intel RealSense D435**

* `/camera/{front|rear}/color/image_raw` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)  
  Color camera images.

* `/camera/{front|rear}/color/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html)  
  Color camera infos.

* `/camera/{front|rear}/aligned_depth_to_color/image_raw` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)  
  Depth camera images.

* `/camera/{front|rear}/aligned_depth_to_color/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html)  
  Depth camera infos.

* `/camera/{front|rear}/depth/color/points` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)  
  Camera point clouds.

* `/camera/{front|rear}/depth/color/cropped/points` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)  
  Cropped versions of point clouds published on `/camera/{front|rear}/depth/color/points`.

**Velodyne VLP-16**

* `/lidar/points` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)  
  LiDAR point cloud.

**Wheel Encoders**

* `/wheel/{left|right}/encoder` ([twizy_wheel_encoder/WheelEncoder](../twizy_wheel_encoder/msg/WheelEncoder.msg))
  Wheel encoder speed measurements.

### Parameters

* `/use_sim_time` (`bool`, default: `false`)  
  See ROS wiki.

**Motor Control**

* `physical/rear_wheel/radius` (`double`, required)  
  Radius of rear wheels.

* `physical/max_forward_speed` (`double`, required)  
  Maximum allowed forward speed of the Twizy.

* `physical/max_reverse_speed` (`double`, required)  
  Maximum allowed reverse speed of the Twizy.

* `physical/max_steering_angle` (`double`, required)  
  Maximum steering angle allowed (at the centerline of the vehicle if using Ackermann steering).

* `physical/wheelbase` (`double`, required)  
  Distance between the Twizy's front and rear axle.

* `physical/front_track` (`double`, required)  
  Distance between centers of the Twizy's front wheels.

* `physical/rear_track` (`double`, required)  
  Distance between centers of the Twizy's rear wheels.

**Piksi Multi**

* `piksi/gyro/cov` (`double`, required)  
  Diagonal elements in the gyro's noise covariance matrix (rad²/s²).

* `piksi/accelerometer/cov"` (`double`, required)  
  Diagonal elements in the accelorometer's noise covariance matrix (m²/s⁴).

* `piksi/gnss/cov` (`double`, required)  
  Diagonal elements in the GNSS' noise covariance matrix (m²).

**Wheel Encoders**

* `physical/rear_wheel/radius` (`double`, required)  
  Radius of rear wheels.

* `~wheel_encoder/cov` (`double`, required)  
  Covariance of speed measurements (m²/s²).

* `~wheel_encoder/ups` (`double`, required)  
  Target frequency to publish wheel encoder messages at.