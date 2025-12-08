from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = "sobit_mini"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("sobit_mini_bringup"),
                    'launch',
                    'robot.launch.py'
                ])

            ]),
            launch_arguments={
                'robot_name': robot_name,
                'enable_gz': 'False',
            }.items()
        ),
    ])
