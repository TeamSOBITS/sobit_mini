import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import yaml
import xacro

def generate_launch_description():
    arg_robot_name = DeclareLaunchArgument('robot_name', default_value='sobit_mini')

    arg_robot_coords_x = DeclareLaunchArgument('robot_coords_x', default_value='0')
    arg_robot_coords_y = DeclareLaunchArgument('robot_coords_y', default_value='0')
    arg_robot_coords_Y = DeclareLaunchArgument('robot_coords_Y', default_value='0')
    
    arg_enable_gz = DeclareLaunchArgument('enable_gz', default_value='True')
    arg_enable_gz_lidar = DeclareLaunchArgument('enable_gz_lidar', default_value='True')
    arg_enable_gz_head_cam_color = DeclareLaunchArgument('enable_gz_head_cam_color', default_value='True')
    arg_enable_gz_head_cam_depth = DeclareLaunchArgument('enable_gz_head_cam_depth', default_value='True')

    return LaunchDescription([
        arg_robot_name,
        arg_robot_coords_x,
        arg_robot_coords_y,
        arg_robot_coords_Y,
        arg_enable_gz,
        arg_enable_gz_lidar,
        arg_enable_gz_head_cam_color,
        arg_enable_gz_head_cam_depth,
        OpaqueFunction(function = launch_gz),
    ])


def launch_gz(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_coords_x = LaunchConfiguration('robot_coords_x').perform(context)
    robot_coords_y = LaunchConfiguration('robot_coords_y').perform(context)
    robot_coords_Y = LaunchConfiguration('robot_coords_Y').perform(context)
    enable_gz = LaunchConfiguration('enable_gz').perform(context)
    enable_gz_lidar = LaunchConfiguration('enable_gz_lidar').perform(context)
    enable_gz_head_cam_color = LaunchConfiguration('enable_gz_head_cam_color').perform(context)
    enable_gz_head_cam_depth = LaunchConfiguration('enable_gz_head_cam_depth').perform(context)
    robot_description = os.path.join(get_package_share_directory(
        'sobit_mini_description'), 
        'robots',
        'sobit_mini.urdf.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={
            'enable_gz' : enable_gz,
            'robot_name' : robot_name,
            'enable_gz_lidar' : enable_gz_lidar,
            'enable_gz_head_cam_color' : enable_gz_head_cam_color,
            'enable_gz_head_cam_depth' : enable_gz_head_cam_depth,
        })


    urg_config = os.path.join(get_package_share_directory("sobit_mini_bringup"), "config", "urg_node_params.yaml")

    kobuki_param_file = os.path.join(get_package_share_directory("sobit_mini_bringup"), "config", "kobuki_node_params.yaml")
    with open(kobuki_param_file, "r") as f:
        kobuki_params = yaml.safe_load(f)["kobuki_ros_node"]["ros__parameters"]


    if enable_gz == 'False':
        controller_config = os.path.join(
            get_package_share_directory(
                'sobit_mini_control'),
                "config", 
                "controllers.yaml"
        )
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=robot_name,
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output="screen",
        )
        kobuki_node = Node(
            package="kobuki_node",
            executable="kobuki_ros_node",
            namespace=robot_name,
            output="both",
            parameters=[kobuki_params]
        )
        urg_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('urg_node'),
                    'launch',
                    'urg.launch.py'
                ])
            ]),
            launch_arguments={
                "config_file" : urg_config,
                "use_namespace" : "true",
                "namespace" : robot_name,
            }.items()
        )
        camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("sobit_mini_bringup"),
                    'launch',
                    'realsense_bringup.launch.py'
                ])
            ]),
        )
        rviz_config = PathJoinSubstitution([
            FindPackageShare('sobit_mini_bringup'),
            'rviz',
            'real.rviz'
        ])

    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
            #  '--use-sim-time',
             'joint_state_broadcaster'
        ],
        output='screen'
    )

    joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active',
             '--controller-manager', robot_name+'/controller_manager',
            #  '--use-sim-time',
             'joint_trajectory_controller'
        ],
        output='screen'
    )

    velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'configured',
             '--controller-manager', robot_name+'/controller_manager',
            #  '--use-sim-time',
             'velocity_controller'
        ],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[
            {"frame_prefix": robot_name + '/'},
            {"robot_description": robot_description_config.toxml()},
            {"use_sim_time": True if enable_gz == 'True' else False},
        ],
        output="screen",
    )

    if enable_gz == 'True':
        gz_spawn_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            namespace=robot_name,
            arguments=[
                '-topic', '/' + robot_name + '/robot_description',
                '-name', robot_name,
                '-x', robot_coords_x,
                '-y', robot_coords_y,
                '-Y', robot_coords_Y,
            ],
            output='screen',
        )

        gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=robot_name,
            arguments=[
                        "/" + robot_name + "/joint_states" + "@sensor_msgs/msg/JointState" + "[ignition.msgs.Model",
                        # "/model/" + robot_name + "/pose" + "@geometry_msgs/msg/Pose" + "[ignition.msgs.Pose",
                        # "/" + robot_name + "/base_front_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/base_front_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_front_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_back_camera/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        # "/" + robot_name + "/base_back_camera/color" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        # "/" + robot_name + "/base_back_camera/depth" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        "/" + robot_name + "/head_camera_base/color/camera_info" + "@sensor_msgs/msg/CameraInfo" + "[ignition.msgs.CameraInfo",
                        "/" + robot_name + "/head_camera_base/color/image_raw" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        "/" + robot_name + "/head_camera_base/depth/image_raw" + "@sensor_msgs/msg/Image" + "[ignition.msgs.Image",
                        "/" + robot_name + "/head_camera_base/depth/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        "/" + robot_name + "/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",

                        "/" + robot_name + "/scan/points" + "@sensor_msgs/msg/PointCloud2" + "[ignition.msgs.PointCloudPacked",
                        "/" + robot_name + "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
                    ],
            output='screen'
        )

        # gz_tf_head_cam_node = Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['--frame-id', robot_name + '/head_camera_depth_optical_frame',
        #                '--child-frame-id', robot_name + '/head_pitch_link/head_camera_depth',
        #                '--pitch', '-1.57',
        #                '--roll', '1.57'],
        #     output='screen',
        # )

        diff_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller',
                '--set-state', 'active',
                '--controller-manager', robot_name+'/controller_manager',
                #  '--use-sim-time',
                'diff_controller'
            ],
            output='screen'
        )

        vel_remap_node = Node(
            package="topic_tools",
            executable="relay",
            name="vel_remap",
            arguments=[f"/{robot_name}/commands/velocity", f"/{robot_name}/diff_controller/cmd_vel_unstamped"]
        )

        odom_remap_node = Node(
            package="topic_tools",
            executable="relay",
            name="odom_remap",
            arguments=[f"/{robot_name}/diff_controller/odom", f"/{robot_name}/odom"]
        )

        rviz_config = PathJoinSubstitution([
            FindPackageShare('sobit_mini_bringup'),
            'rviz',
            'gazebo.rviz'
        ])


    action_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sobit_mini_library'),
                'launch',
                'library_server.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'enable_gz': enable_gz,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name=robot_name+'_rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    if enable_gz == 'False':
        return {
            kobuki_node,
            urg_node,
            camera_node,
            ros2_control_node,
            joint_state_broadcaster,
            velocity_controller,
            joint_trajectory_controller,
            robot_state_publisher_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[action_server_launch],
                )
            ),
            rviz_node,
        }
    
    else:
        return [
            gz_spawn_entity_node,
            gz_bridge_node,
            # gz_tf_head_cam_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity_node,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[joint_trajectory_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[velocity_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[diff_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[vel_remap_node],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[odom_remap_node],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[action_server_launch],
                )
            ),
            robot_state_publisher_node,
            rviz_node,
        ]