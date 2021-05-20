# twizy_bringup <!-- omit in toc -->

This package contains launch files for control of an autonomous Renault Twizy at the Department of Electrical Engineering, Chalmers University of Technology, Sweden.

**Package Links**

* [Code API](https://ossianeriksson.github.io/autonomous-twizy/twizy_bringup/html/index.html)

**Contents**

- [Launch Files](#launch-files)

# Launch Files

See launch files in [launch](launch). Here is a quick guide to an excerpt of the available launch files:

* [navigation.launch](launch/navigation.launch)  
  Main launch file for running on the physical car. Nodes in this file are expected to run on the Twizy's on-board computer. To give the car a goal to drive towards through a GUI you can then start rviz separately (manually, possibly in another terminal window) on your local computer, see [rviz.launch](launch/include/rviz.launch). Communication between rviz on your local computer and the rest of the ROS-nodes on the on-board computer is then accomplished according to [MultipleMachines on the ROS wiki](http://wiki.ros.org/ROS/Tutorials/MultipleMachines).

* [webots_keyop.launch](launch/webots_keyop.launch)  
  Demo launch file showing the car driving locally in simulation by keyboard control. This launch file might not run on the car's on-board computer since webots might not be installed on there.

* [webots_navigation.launch](launch/webots_keyop.launch)  
  Main launch file for twizy simulation. This file starts every node you need to simulate the car locally in Webots in a way which is meant to simulate running [navigation.launch](launch/navigation.launch) on the actual car. This launch file might not run on the car's on-board computer since webots might not be installed on there.

* [rviz.launch](launch/include/rviz.launch)  
  Starts rviz using the configuration contained in this package for visualisation of the Twizy.