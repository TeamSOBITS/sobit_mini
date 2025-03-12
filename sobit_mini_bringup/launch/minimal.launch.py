# Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

import yaml 
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.actions import Node



def generate_launch_description():
    robot_name = "sobit_mini"
    bringup_pkg = robot_name + "_bringup"

    rviz_config = os.path.join(get_package_share_directory(
        bringup_pkg), "rviz", "real.rviz")

    urg_config = os.path.join(get_package_share_directory(
        bringup_pkg), "config", "urg_node_params.yaml")
    
    kobuki_param_file = os.path.join(get_package_share_directory("sobit_mini_bringup"), "config", "kobuki_node_params.yaml")
    with open(kobuki_param_file, "r") as f:
        kobuki_params = yaml.safe_load(f)["kobuki_ros_node"]["ros__parameters"]

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([os.path.join(
                    get_package_share_directory('sobit_mini_bringup'),
                    'launch',
                    'robot.launch.py')
                ])

            ]),
            launch_arguments={
                'robot_name': 'sobit_mini',
                'robot_coords_x': '0', # x 
                'robot_coords_y': '0', # y
                'robot_coords_Y': '0', # yaw
            }.items()
        ),
        Node(
            package="kobuki_node",
            executable="kobuki_ros_node",
            output="both",
            parameters=[kobuki_params]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([os.path.join(
                    get_package_share_directory('sobit_mini_bringup'),
                    'launch',
                    'realsense_bringup.launch.py')
                ])

            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    get_package_share_directory('urg_node'),
                    'launch',
                    'urg.launch.py'
                ])
            ]),
            launch_arguments={
                "config_file" : urg_config
            }.items()
        )
    ])
