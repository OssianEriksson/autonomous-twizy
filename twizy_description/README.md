# twizy_description <!-- omit in toc -->

This package contains files and code neccessary for the description of an autonomous Renault Twizy at the Department of Electrical Engineering, Chalmers University of Technology, Sweden.

**Package Links**

* [Code API](https://ossianeriksson.github.io/autonomous-twizy/twizy_description/html/index.html)

**Contents**

- [Launch Files](#launch-files)

# Directory Structure

* [config/](config) contains YAML files of with attributes of the Twizy, e.g.
  
  * [model.yaml](config/model.yaml) contains parameters neccesary for the construction of the twizy model but are otherwise irrelevant or accessable elseware when the system is running
 
  * [physical.yaml](config/physical.yaml) contains physical attributes of the Twizy. This file is separate from [config/model.yaml](config/model.yaml) to discorage users from uploading [config/model.yaml](config/model.yaml) to the ROS parameter server as the values contained here should be accessed in other ways

* [meshes/](meshes) contains meshes and textures for visualization purposes

* [robot/](robot) contains models of robots created using the `robot_description` python module which is described in the [code API](https://ossianeriksson.github.io/autonomous-twizy/twizy_description/html/index.html)

  * [twizy.py](robot/twizy.py) generates models of the Twizy in URDF and Webots PROTO formats