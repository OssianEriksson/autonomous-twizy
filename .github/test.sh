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

echo -e "\n**************************************************\n"
rostest twizy_control test_twizy_can_control.test --text
echo -e "\n**************************************************\n"

# Run unit and integration tests
catkin_make run_tests
# Needed for exit code (and debugging)
catkin_test_results --verbose