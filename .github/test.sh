#!/bin/bash

# Fail if any sub-command fails
set -e

# Installs the repo, ROS, ROS packages and other required dependencies
"${GITHUB_WORKSPACE}/src/install.sh"
source /opt/ros/$ROS_DISTRO/setup.bash

cd ~/autonomous-twizy

# Build all packages in the workspace
catkin_make
source devel/setup.bash

ROS_HOSTNAME=localhost
ROS_MASTER_URI="http://localhost:11311"

# Run unit and integration tests
catkin_make run_tests
# Needed for exit code (and debugging)
catkin_test_results --verbose