#!/bin/bash

# Fail if any sub-command fails
set -e

function append_to_bashrc {
	# Appends $1 to .bashrc if it has not already been added and the script is not running as part of a Github action
	if ! grep -qF "$1" ~/.bashrc && [[ -z "${GITHUB_ACTION}" ]]; then
		echo "$1" >> ~/.bashrc
	fi
}


# Default value for ROS_DISTRO is noetic
ROS_DISTRO=${ROS_DISTRO:-noetic}

if [[ -z "${GITHUB_ACTION}" ]]; then # If running on local computer 
	# Clone the repo into what will become the catkin workspace if no prior clone exists at the target location
	if ! git -C ~/autonomous-twizy/src branch > /dev/null && ! git clone git@github.com:OssianEriksson/autonomous-twizy.git ~/autonomous-twizy/src; then
		exit 1
	fi
else # If running as Github action
	# Create symlinks to clone location
	mkdir -p ~
	ln -s "${GITHUB_WORKSPACE}" ~/autonomous-twizy
fi

# Install ROS according to http://wiki.ros.org/noetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
if [[ -z "${GITHUB_ACTION}" ]]; then # If running on local computer 
	sudo apt-get install -y ros-$ROS_DISTRO-desktop-full
else # If running as Github action
	sudo apt-get install -y ros-$ROS_DISTRO-ros-base
fi
append_to_bashrc "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/ros/$ROS_DISTRO/setup.bash

# Install rosdep and ROS packages
sudo apt-get install -y python3-rosdep
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
rosdep install -i --from-path ~/autonomous-twizy/src --rosdistro $ROS_DISTRO -y

# Install Kvaser's canlib according to https://www.kvaser.com/canlib-webhelp/section_install_linux.html#Installing-the-driver
sudo apt-get install -y build-essential
cd /tmp
wget --content-disposition "https://www.kvaser.com/downloads-kvaser/?utm_source=software&utm_ean=7330130980754&utm_status=latest"
tar xvzf linuxcan.tar.gz
cd linuxcan
make
sudo make install

# Install python packages not indexed by rosdep
sudo apt-get install -y python3-pip
# sbp at the time of writing requires a newer version of numpy than provided by rosdep, so override with a manuall install of numpy with pip3
pip3 install -U canlib sbp==3.4.6 numpy==1.20.2

if [[ -z "${GITHUB_ACTION}" ]] && [[ -z "${TWIZY_ONBOARD}" ]]; then # If running on local computer which is not the Twizy's on-board computer
	# Install webots according to https://www.cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt
	wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
	sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
	sudo apt-get update
	sudo apt-get install -y webots
	append_to_bashrc "export WEBOTS_HOME=/usr/local/webots"
fi

if [[ ! -z "${TWIZY_ONBOARD}" ]]; then # If running on the Twizy's on-board computer
	append_to_bashrc "export TWIZY_ONBOARD=1"
fi

# Automatically source the workspace's setup.bash in new shells
append_to_bashrc "[ -f ~/autonomous-twizy/devel/setup.bash ] && source ~/autonomous-twizy/devel/setup.bash"

if [[ -z "${GITHUB_ACTION}" ]]; then # If running on a local computer
	echo -e "\n\nInstallation completed!"
	echo -e "\nThe repository has been cloned to ~/autonomous-twizy/src, and the catkin workspace is located at ~/autonomous-twizy. If you have trouble building or running please read through the installation log above and look for errors"
	echo -e "\nFinally, please run"
	echo -e "\n    source ~/.bashrc\n"
fi
