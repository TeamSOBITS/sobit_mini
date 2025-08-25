#!/bin/bash

echo "╔══╣ Setup: SOBIT MINI (STARTING) ╠══╗"

DIR=$(pwd)
cd ..

echo "Cloning: sobits_interfaces"
git clone -b "$ROS_DISTRO-devel" "https://github.com/TeamSOBITS/sobits_interfaces.git"

if [[ -f "sobits_interfaces/install.sh" ]]; then
    echo "Running install.sh in sobits_interfaces."
    cd "sobits_interfaces" || exit
    bash install.sh
    cd ..
fi

echo "Cloning: dynamixel_hardware"
git clone -b feature/multi_control "https://github.com/TeamSOBITS/dynamixel_hardware.git"

if [[ -f "dynamixel_hardware/install.sh" ]]; then
    echo "Running install.sh in dynamixel_hardware."
    cd "dynamixel_hardware" || exit
    bash install.sh
    cd ..
fi

echo "Cloning: realsense_ros"
git clone -b "$ROS_DISTRO-devel" "https://github.com/TeamSOBITS/realsense_ros.git"

if [[ -f "realsense_ros/install.sh" ]]; then
    echo "Running install.sh in realsense_ros."
    cd "realsense_ros" || exit
    bash install.sh
    cd ..
fi

echo "Cloning: urg_node"
git clone -b "$ROS_DISTRO-devel" "https://github.com/TeamSOBITS/urg_node.git"

if [[ -f "urg_node/install.sh" ]]; then
    echo "Running install.sh in urg_node."
    cd "urg_node" || exit
    bash install.sh
    cd ..
fi

echo "Cloning: kobuki_ros"
git clone -b "$ROS_DISTRO-devel" "https://github.com/TeamSOBITS/kobuki_ros.git"

if [[ -f "kobuki_ros/install.sh" ]]; then
    echo "Running install.sh in kobuki_ros."
    cd "kobuki_ros" || exit
    bash install.sh
    cd ..
fi

echo "Cloning: turtlebot2_description"
git clone -b "$ROS_DISTRO-devel" "https://github.com/TeamSOBITS/turtlebot2_description.git"

if [[ -f "turtlebot2_description/install.sh" ]]; then
    echo "Running install.sh in turtlebot2_description."
    cd "turtlebot2_description" || exit
    bash install.sh
    cd ..
fi

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
    ros-"$ROS_DISTRO"-ign-ros2-control-demos

echo "╚══╣ Setup: SOBIT MINI (FINISHED) ╠══╝"