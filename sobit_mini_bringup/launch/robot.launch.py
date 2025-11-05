import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_mini')
    return LaunchDescription([
        arg_robot_name,
        OpaqueFunction(function=launch_gz),
    ])

def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    robot_description_path = os.path.join(
        get_package_share_directory('sobit_mini_description'),
        'robots',
        'sobit_mini.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description_path,
        mappings={'robot_name': robot_name}
    )

    controller_pkg = robot_name + "_control"
    controller_config = os.path.join(
        get_package_share_directory(controller_pkg),
        "config",
        "controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_config.toxml()},
            controller_config
        ],
        output="screen",
        namespace=robot_name,
    )

    joint_state_broadcaster = TimerAction(
        period=1.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'control', 'load_controller',
                '--set-state', 'active',
                '--controller-manager', f'{robot_name}/controller_manager',
                'joint_state_broadcaster'
            ],
            output='screen'
        )]
    )

    velocity_controller = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'control', 'load_controller',
                '--set-state', 'configured',
                '--controller-manager', f'{robot_name}/controller_manager',
                'velocity_controller'
            ],
            output='screen'
        )]
    )

    joint_trajectory_controller = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'control', 'load_controller',
                '--set-state', 'active',
                '--controller-manager', f'{robot_name}/controller_manager',
                'joint_trajectory_controller'
            ],
            output='screen'
        )]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[
            {"frame_prefix": robot_name + '/'},
            {"robot_description": robot_description_config.toxml()},
        ],
        output="screen",
    )

    return [
        ros2_control_node,
        joint_state_broadcaster,
        velocity_controller,
        joint_trajectory_controller,
        robot_state_publisher_node,
    ]
