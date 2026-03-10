#!/bin/bash

echo "╔══╣ Setup: SOBIT MINI (STARTING) ╠══╗"

DIR=$(pwd)
cd ..

ros_packages=(
    "sobits_interfaces"
    "dynamixel_hardware"
    "realsense_ros"
    "urg_node"
    "turtlebot2_ros2"
    "sobits_gazebo_worlds"
)

for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone -b $ROS_DISTRO-devel https://github.com/TeamSOBITS/${ros_packages[i]}.git

    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}

cd "$DIR"

# Download required dependencies
python3 -m pip install --break-system-packages \
    transforms3d

sudo apt update
sudo apt install -y \
    ros-"$ROS_DISTRO"-ecl-linear-algebra \
    ros-"$ROS_DISTRO"-ecl-geometry \
    ros-"$ROS_DISTRO"-kobuki-ros-interfaces \
    ros-"$ROS_DISTRO"-laser-proc \
    ros-"$ROS_DISTRO"-urg-c \
    ros-"$ROS_DISTRO"-urg-node \
    ros-"$ROS_DISTRO"-urg-node-msgs \
    ros-"$ROS_DISTRO"-robot-state-publisher \
    ros-"$ROS_DISTRO"-joint-state-publisher \
    ros-"$ROS_DISTRO"-joint-state-publisher-gui \
    ros-"$ROS_DISTRO"-joint-limits \
    ros-"$ROS_DISTRO"-hardware-interface \
    ros-"$ROS_DISTRO"-transmission-interface \
    ros-"$ROS_DISTRO"-controller-interface \
    ros-"$ROS_DISTRO"-controller-manager \
    ros-"$ROS_DISTRO"-controller-manager-msgs \
    ros-"$ROS_DISTRO"-tf2 \
    ros-"$ROS_DISTRO"-tf2-ros \
    ros-"$ROS_DISTRO"-sensor-msgs \
    ros-"$ROS_DISTRO"-trajectory-msgs \
    ros-"$ROS_DISTRO"-geometry-msgs \
    ros-"$ROS_DISTRO"-joy \
    ros-"$ROS_DISTRO"-joy-linux \
    ros-"$ROS_DISTRO"-ros2-control \
    ros-"$ROS_DISTRO"-ros2-controllers \
    ros-"$ROS_DISTRO"-control-toolbox \
    ros-"$ROS_DISTRO"-position-controllers \
    ros-"$ROS_DISTRO"-velocity-controllers \
    ros-"$ROS_DISTRO"-effort-controllers \
    ros-"$ROS_DISTRO"-joint-trajectory-controller \
    ros-"$ROS_DISTRO"-joint-state-broadcaster \
    ros-"$ROS_DISTRO"-urdf \
    ros-"$ROS_DISTRO"-urdf-launch \
    ros-"$ROS_DISTRO"-xacro \
    ros-"$ROS_DISTRO"-tf-transformations \
    ros-"$ROS_DISTRO"-gz-ros2-control \
    ros-"$ROS_DISTRO"-actuator-msgs \
    ros-"$ROS_DISTRO"-gps-msgs \
    ros-"$ROS_DISTRO"-ros-gz \
    ros-"$ROS_DISTRO"-ros-gz-bridge \
    ros-"$ROS_DISTRO"-ros-gz-sim \
    ros-"$ROS_DISTRO"-ros-gz-interfaces \
    ros-"$ROS_DISTRO"-topic-tools \
    ros-"$ROS_DISTRO"-twist-stamper

# Set up environment variables
echo "" >> /home/$USERNAME/.bashrc
echo "# SOBIT MINI environment variables" >> /home/$USERNAME/.bashrc
echo "export DXL_SM_PORT=`realpath /dev/serial/by-id/usb-BestTechnology_E143_E143-if00-port0`" >> /home/$USERNAME/.bashrc
echo "export KOBUKI_SM_PORT=`realpath /dev/serial/by-id/usb-Yujin_Robot_iClebo_Kobuki_kobuki_AI06FCC2-if00-port0`" >> /home/$USERNAME/.bashrc
echo "export HOKUYO_SM_PORT=`realpath /dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00`" >> /home/$USERNAME/.bashrc
echo "" >> /home/$USERNAME/.bashrc
source /home/$USERNAME/.bashrc

# # Reload udev rules
sudo udevadm control --reload-rules

# # Trigger the new rules
sudo udevadm trigger


# Go back to previous directory
cd ${DIR}

echo "╚══╣ Setup: SOBIT MINI (FINISHED) ╠══╝"
