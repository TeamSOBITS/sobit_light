#!/bin/bash

set -u  
set -o pipefail
echo "╔══╣ Setup: SOBIT LIGHT (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# Download required packages for SOBIT LIGHT
ros_packages=(
    "sobits_interfaces"
    "sobits_robot_descriptor"
    "dynamixel_hardware"
    "realsense_ros"
    "kachaka-api"
    "sobits_gazebo_worlds"
    "sobits_viz"
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

# Clone patched MoveIt2 (sparse checkout: only patched packages)
if [ ! -d "moveit2_patch" ]; then
    echo "Cloning: moveit2_patch (sparse checkout)"
    git clone --sparse --branch fix/robot-interaction-frame-prefix \
        https://github.com/TeamSOBITS/moveit2.git moveit2_patch
    cd moveit2_patch
    git sparse-checkout set \
        moveit_ros/robot_interaction \
        moveit_ros/visualization
    cd ..
else
    echo "moveit2_patch already exists, skipping clone."
fi

# Go back to previous directory
cd ${DIR}

# Download required dependencies
python3 -m pip install --break-system-packages \
    transforms3d

# Install every ROS dependency declared in the package.xml files through rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

rosdep_paths=(
    "${DIR}"
    "${DIR}/../kachaka-api/ros2/kachaka_description"
    "${DIR}/../kachaka-api/ros2/kachaka_interfaces"
    "${DIR}/../moveit2_patch"
)
for pkg in "${ros_packages[@]}" tmc_wrs_gz gz_human_sim aws_small_house_world; do
    [ "${pkg}" == "kachaka-api" ] && continue
    [ -d "${DIR}/../${pkg}" ] && rosdep_paths+=("${DIR}/../${pkg}")
done
sudo apt-get update
rosdep install -r -y -i --from-paths "${rosdep_paths[@]}"

# Set up environment variables
echo "" >> /home/$USERNAME/.bashrc
echo "# SOBIT LIGHT environment variables" >> /home/$USERNAME/.bashrc
echo "export DXL_SL_PORT=`realpath /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISSV2-if00-port0`" >> /home/$USERNAME/.bashrc
echo "" >> /home/$USERNAME/.bashrc
source /home/$USERNAME/.bashrc

# # Reload udev rules
sudo udevadm control --reload-rules ||true

# # Trigger the new rules
sudo udevadm trigger ||true

# Go back to previous directory
cd ${DIR}


echo "╚══╣ Setup: SOBIT LIGHT (FINISHED) ╠══╝"
