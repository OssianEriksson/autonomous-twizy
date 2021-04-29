#!/bin/bash

# Fail if any sub-command fails
set -e

# Installs the repo, ROS, ROS packages and other required dependencies
"${GITHUB_WORKSPACE}/src/install.sh"
source /opt/ros/$ROS_DISTRO/setup.bash

cd ~/autonomous-twizy

# Build all packages in the workspace
catkin_make -DCATKIN_BLACKLIST_PACKAGES="twizy_webots"
source devel/setup.bash

# Run unit and integration tests
catkin_make -DCATKIN_BLACKLIST_PACKAGES="twizy_webots" run_tests
# Needed for exit code (and debugging)
catkin_test_results --verbose