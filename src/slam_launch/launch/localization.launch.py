#!/usr/bin/env python3
"""
Localization Launch File - Uses a saved map for localization

Usage:
    ros2 launch slam_launch localization.launch.py map:=/path/to/my_map.posegraph
"""

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():
    pkg_slam_launch = get_package_share_directory('slam_launch')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    map_file = LaunchConfiguration('map')
    loc_params = LaunchConfiguration('params')

    # RPLIDAR A3 Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidarNode',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
    )

    # RF2O Laser Odometry Node
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': True,
            'base_frame_id': 'laser',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
    )

    # SLAM Toolbox in Localization Mode
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            loc_params,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file,
            }
        ],
    )

    # Auto-activate lifecycle node
    activate_on_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_toolbox_node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    ))

    slam_delayed = TimerAction(
        period=3.0,
        actions=[slam_toolbox_node, activate_on_configure, configure_event]
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_54a7b95f40534148bc1423cfb3402cbc-if00-port0',
            description='RPLIDAR serial port'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='/home/peru0002/ros2_ws/maps/my_map',
            description='Path to map file (without extension)'
        ),
        DeclareLaunchArgument(
            'params',
            default_value=PathJoinSubstitution([pkg_slam_launch, 'config', 'localization.yaml']),
            description='Path to localization configuration file'
        ),

        # Nodes
        rplidar_node,
        rf2o_node,
        slam_delayed,
    ])
