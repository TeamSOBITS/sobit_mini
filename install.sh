#!/bin/bash

cd $(pwd)/..

# git cloneしたいTeamSOBITSのROSパッケージを以下に記述
ros_packages=(
    "sobit_common" \
    "sobits_msgs" \
    "urg_node"\
    "realsense_ros"\
    "turtlebot2_on_noetic"
)

for ((i = 0; i < ${#ros_packages[@]}; i++)) {
    # echo "array[$i] = ${array[i]}"
    echo "${ros_packages[i]}"
    git clone https://github.com/TeamSOBITS/${ros_packages[i]}.git
}

cd 

# Setting Sound configure
echo "pacmd load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket &> /dev/null" >> ~/.bashrc
echo "#!bin/bash
touch /tmp/pulseaudio.client.conf
echo \"default-server = unix:/tmp/pulseaudio.socket \n 
      # Prevent a server running in the container \n 
      autospawn = no \n 
      daemon-binary = /bin/true \n
      # Prevent the use of shared memory \n
      enable-shm = false\" >> /tmp/pulseaudio.client.conf" | sudo tee /etc/profile.d/sound_setup.sh
sudo bash /etc/profile.d/sound_setup.sh

# Seting dynamixel USB1 (SOBIT PRO arm_pantilt)
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", SYMLINK+=\"input/dynamixel1\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dynamixel1.rules
# echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", ATTRS{serial}==\"E143\", SYMLINK+=\"input/dongle\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dongle.rules
# sudo /etc/init.d/udev reload

# Seting dynamixel USB2 (SOBIT PRO wheel)
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", SYMLINK+=\"input/dynamixel2\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dynamixel2.rules
# echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", ATTRS{serial}==\"E148\", SYMLINK+=\"input/dxhub\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/dxhub.rules
# sudo /etc/init.d/udev reload

# Seting ps4_joy_control USB
echo "KERNEL==\"uinput\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"05c4\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:05C4.*\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"054c\", ATTRS{idProduct}==\"09cc\", MODE=\"0666\"
      KERNEL==\"hidraw*\", SUBSYSTEM==\"hidraw\", KERNELS==\"0005:054C:09CC.*\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/50-ds4drv.rules
# sudo /etc/init.d/udev reload

sudo udevadm control --reload-rules
sudo udevadm trigger

# USB Reload
sudo /etc/init.d/udev reload