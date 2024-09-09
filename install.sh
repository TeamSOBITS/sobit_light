#!/bin/bash

echo "╔══╣ Setup: SOBIT LIGHT (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download required packages for SOBIT LIGHT
ros_packages=(
    # "sobit_common" \
    "sobits_msgs" \
    # "urg_node" \
    "realsense_ros" \
    # "kachaka_api"
)

#Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    # git clone -b $ROS_DISTRO https://github.com/TeamSOBITS/${ros_packages[i]}.git
    git clone -b feature/humble-devel https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}

# Go back to previous directory
cd ${DIR}

# Download required dependencies
# sudo apt-get update
# sudo apt-get install -y \
#     mpg321 

# Download ROS packages
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-control-toolbox \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-position-controllers \
    ros-$ROS_DISTRO-velocity-controllers \
    ros-$ROS_DISTRO-effort-controllers \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-joint-group-impedance-controller \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-limits \
    ros-$ROS_DISTRO-robot-controllers \
    ros-$ROS_DISTRO-robot-controllers-interface \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-transmission-interface \
    ros-$ROS_DISTRO-urdf \
    ros-$ROS_DISTRO-urdf-launch \
    ros-$ROS_DISTRO-xacro

# Install Gazebo Fortress
# - Install some necessary tools
# sudo apt-get update
# sudo apt-get install lsb-release gnupg

# # - Install Ignition Fortress
# sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# sudo apt-get update
# sudo apt-get install ignition-fortress

# ROS2 Control Gazebo Plugins
# cd ..
# git clone https://github.com/ros-controls/gz_ros2_control/ -b ${ROS_DISTRO}
# rosdep update
# rosdep install --from-paths gz_ros2_control/ -i -y --rosdistro ${ROS_DISTRO}

# Install Gazebo Fortress with binaries
sudo apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ign-ros2-control \
    ros-${ROS_DISTRO}-ign-ros2-control-demos


# # Setting up Dynamixel USB configuration (SOBIT LIGHT: Head and Arm Robot Mechanism)
# echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", SYMLINK+=\"input/dx_upper\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dx_upper.rules

# # Setting up PS4 Joystick USB configuration
# echo "KERNEL==\"uinput\", MODE=\"0666\"
#       KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"05c4\", MODE=\"0666\"
#       KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:05C4.*\", MODE=\"0666\"
#       KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"09cc\", MODE=\"0666\"
#       KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:09CC.*\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/50-ds4drv.rules

# # Reload udev rules
# sudo udevadm control --reload-rules

# # Trigger the new rules
# sudo udevadm trigger

# Go back to previous directory
cd ${DIR}


echo "╚══╣ Setup: SOBIT LIGHT (FINISHED) ╠══╝"
