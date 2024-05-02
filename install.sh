#!/bin/bash

echo "╔══╣ Setup: SOBIT MINI (STARTING) ╠══╗"


# Keep track of the current directory
DIR=`pwd`
cd ..

# git cloneしたいTeamSOBITSのROSパッケージを以下に記述
ros_packages=(
    "sobit_common" \
    "sobits_msgs" \
    "urg_node"\
    "realsense_ros"\
    "turtlebot2_on_noetic"
)

for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    echo "Clonning: ${ros_packages[i]}"
    git clone https://github.com/TeamSOBITS/${ros_packages[i]}.git

    # Check if install.sh exists in each package
    if [ -f ${ros_packages[i]}/install.sh ]; then
        echo "Running install.sh in ${ros_packages[i]}."
        cd ${ros_packages[i]}
        bash install.sh
        cd ..
    fi
}

# Setup Turtlebot2 (Kobuki) for ROS Noetic
cd ${DIR}
bash ../turtlebot2_on_noetic/turtlebot/setup_kobuki.sh

# Download ROS packages
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-pybind11-catkin \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-controller \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-joint-limits-interface \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-transmission-interface \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-trajectory-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-joy

# Setting up Dynamixel USB configuration (SOBIT MINI: Head and Arm Robot Mechanism)
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", SYMLINK+=\"input/dx_upper\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dx_upper.rules

# Setting up PS4/PS3 Joystick USB configuration
echo "KERNEL==\"uinput\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"05c4\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:05C4.*\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"09cc\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:09CC.*\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/50-ds4drv.rules

# Setting up kobuki rules
echo "# On precise, for some reason, USER and GROUP are getting ignored.
      # So setting mode = 0666 for now.
      SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6001\", ATTRS{serial}==\"kobuki*\", 
      ATTR{device/latency_timer}=\"1\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"input/kobuki\", KERNEL==\"ttyUSB*\"
      # Bluetooth module (currently not supported and may have problems)
      # SUBSYSTEM==\"tty\", ATTRS{address}==\"00:00:00:41:48:22\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"input/kobuki\"" | sudo tee /etc/udev/rules.d/57-kobuki.rules

# Reload udev rules
sudo udevadm control --reload-rules

# Trigger the new rules
sudo udevadm trigger


echo "╚══╣ Setup: SOBIT MINI (FINISHED) ╠══╝"
