[![platform](https://img.shields.io/badge/platform-ubuntu%2020.04-blue)](https://releases.ubuntu.com/20.04/)
[![ros](https://img.shields.io/badge/ROS-noetic-blue)](http://wiki.ros.org/noetic)
[![Catkin Tests](https://github.com/OssianEriksson/autonomous-twizy/actions/workflows/catkin_tests.yaml/badge.svg)](https://github.com/OssianEriksson/autonomous-twizy/actions/workflows/catkin_tests.yaml)
[![Code API Docs](https://github.com/OssianEriksson/autonomous-twizy/actions/workflows/code_api_docs.yaml/badge.svg)](https://github.com/OssianEriksson/autonomous-twizy/actions/workflows/code_api_docs.yaml)

# autonomous-twizy

This repository contains ROS packages for control of an autonomous Renault Twizy at the Department of Electrical Engineering, Chalmers University of Technology, Sweden.

# Quick start

For more detailed information see the [wiki](https://github.com/OssianEriksson/autonomous-twizy/wiki/Getting-Started).

1. Installing

   ```sh
   git clone git@github.com:OssianEriksson/autonomous-twizy.git ~/autonomous-twizy/src
   ~/autonomous-twizy/src/install.sh
   source ~/.bashrc
   ```

2. Building

   ```sh
   cd ~/autonomous-twizy
   catkin_make
   source devel/setup.bash
   ```

3. Running Example Launch Files
   
   Keyboard control demo in webots:
   ```sh
   roslaunch twizy_examples webots_keyop.launch
   ```
   
   For more example launch files please see [twizy_examples](twizy_examples).

# API Documentation

## Python/C++ API

View pre-built code API on [Github pages](https://ossianeriksson.github.io/autonomous-twizy/) or build locally with
```sh
~/autonomous-twizy/src/build_docs.sh
```

## ROS API

See READMEs in root directory of the corresponding ROS package

# Licence

Distributed under the Apache 2.0 License. See [LICENSE](LICENSE) for more information.
