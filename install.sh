#!/bin/bash

echo "╔══╣ Setup: SOBIT MINI (STARTING) ╠══╗"

DIR=$(pwd)
cd ..

ros_packages=(
    "sobits_interfaces" \
    "dynamixel_hardware" \
    "realsense_ros" \
    "urg_node" \
    "kobuki_ros" \
    "turtlebot2_description" \
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

cd "$DIR" || exit

python3 -m pip install transforms3d

sudo apt-get update
sudo apt-get install -y \
    ros-"$ROS_DISTRO"-ecl-linear-algebra \
    ros-"$ROS_DISTRO"-kobuki-ros-interfaces \
    ros-"$ROS_DISTRO"-kobuki-core \
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
    ros-"$ROS_DISTRO"-tf2 \
    ros-"$ROS_DISTRO"-tf2-ros \
    ros-"$ROS_DISTRO"-sensor-msgs \
    ros-"$ROS_DISTRO"-trajectory-msgs \
    ros-"$ROS_DISTRO"-geometry-msgs \
    ros-"$ROS_DISTRO"-joy \
    ros-"$ROS_DISTRO"-ros2-control \
    ros-"$ROS_DISTRO"-ros2-controllers \
    ros-"$ROS_DISTRO"-control-toolbox \
    ros-"$ROS_DISTRO"-position-controllers \
    ros-"$ROS_DISTRO"-velocity-controllers \
    ros-"$ROS_DISTRO"-effort-controllers \
    ros-"$ROS_DISTRO"-joint-trajectory-controller \
    ros-"$ROS_DISTRO"-joint-group-impedance-controller \
    ros-"$ROS_DISTRO"-joint-state-broadcaster \
    ros-"$ROS_DISTRO"-robot-controllers \
    ros-"$ROS_DISTRO"-robot-controllers-interface \
    ros-"$ROS_DISTRO"-urdf \
    ros-"$ROS_DISTRO"-urdf-launch \
    ros-"$ROS_DISTRO"-xacro \
    ros-"$ROS_DISTRO"-tf-transformations

sudo apt-get install -y \
    ros-"$ROS_DISTRO"-ros-gz \
    ros-"$ROS_DISTRO"-ign-ros2-control \
    ros-"$ROS_DISTRO"-ign-ros2-control-demos \
    ros-"$ROS_DISTRO"-topic-tools

echo "╚══╣ Setup: SOBIT MINI (FINISHED) ╠══╝"
