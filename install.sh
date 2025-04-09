#!/bin/bash

echo "╔══╣ Setup: SOBIT LIGHT (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download required packages for SOBIT LIGHT
ros_packages=(
    # "sobits_msgs" \
    # "dynamixel_hardware" \
    # "realsense_ros" \
    "kachaka-api"
)

#Clone all packages
for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone -b $ROS_DISTRO-devel https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
    # If kachaka-api, delete kachaka_grpc_ros2_bridge
    if [ ${ros_packages[i]} == "kachaka-api" ]; then
        echo "Deleting kachaka_grpc_ros2_bridge"
        rm -rf kachaka-api/ros2/kachaka_grpc_ros2_bridge
    fi
}

# Go back to previous directory
cd ${DIR}

# Download required dependencies
python3 -m pip install \
    transforms3d

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
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-std-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-trajectory-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-launch \
    ros-$ROS_DISTRO-launch-ros

# Install Gazebo Harmonic with binaries
# sudo apt-get update
# sudo apt-get install -y \
#     curl \
#     lsb-release gnupg

# sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# sudo apt-get update
# sudo apt-get install -y \
#     gz-harmonic


# Install Gazebo Fortress with binaries
sudo apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ign-ros2-control \
    ros-${ROS_DISTRO}-ign-ros2-control-demos


# # Setting up Dynamixel USB configuration (SOBIT LIGHT: Head and Arm Robot Mechanism)
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", ATTRS{serial}==\"FT8ISSV2\", SYMLINK+=\"ttyUSB-DXL_light\", MODE=\"0666\", GROUP:=\"dialout\"," | sudo tee /etc/udev/rules.d/dxl_light.rules
sudo usermod -aG dialout $USER

# # Setting up PS4 Joystick USB configuration
# echo "KERNEL==\"uinput\", MODE=\"0666\"
#       KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"05c4\", MODE=\"0666\"
#       KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:05C4.*\", MODE=\"0666\"
#       KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"09cc\", MODE=\"0666\"
#       KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:09CC.*\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/50-ds4drv.rules

# # Reload udev rules
sudo udevadm control --reload-rules

# # Trigger the new rules
sudo udevadm trigger

# Go back to previous directory
cd ${DIR}


echo "╚══╣ Setup: SOBIT LIGHT (FINISHED) ╠══╝"
