import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = 'sobit_mini'
    robot_id = 0

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
                    "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
                   ],
        output='screen'
    )

    world_file = os.path.join(get_package_share_directory(
        'sobits_gazebo_worlds'), 
        'worlds',
        'rcjo2025_arena.world.xacro'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args' : ' -r -v 4 ' + world_file,
            }.items()
        ),
        gz_bridge_node,
        # Launch Robot No. 1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sobit_mini_bringup'),
                    'launch',
                    'robot.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_name': robot_name if robot_id == 0 else robot_name + '_' + str(robot_id),
                'robot_coords_x': '-5.5', # x 
                'robot_coords_y': '1.5',  # y
                'robot_coords_Y': '0.0',  # yaw
                'enable_gz_lidar' : 'True',
                'enable_gz_head_cam_color': 'True',
                'enable_gz_head_cam_depth': 'True',
                'enable_gz' : 'True',
            }.items()
        ),
        # Launch Robot No. 2
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('sobit_mini_bringup'),
        #             'launch',
        #             'robot.launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'robot_name': robot_name if (robot_id+1) == 0 else robot_name + '_' + str(robot_id+1),
        #         'robot_coords_x': '-5.5', # x 
        #         'robot_coords_y': '-2.5', # y
        #         'robot_coords_Y': '0.0',  # yaw
        #         'enable_gz_lidar' : 'True',
        #         'enable_gz_head_cam_color': 'True',
        #         'enable_gz_head_cam_depth': 'True',
        #         'enable_gz': 'True',
        #     }.items()
        # ),
    ])
