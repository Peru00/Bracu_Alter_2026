#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # SLAM Toolbox configuration
    slam_params = {
        'use_sim_time': use_sim_time,
        'solver_plugin': 'solver_plugins::CeresSolver',
        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
        'ceres_preconditioner': 'SCHUR_JACOBI',
        'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
        'ceres_dogleg_type': 'TRADITIONAL_DOGLEG',
        'ceres_loss_function': 'None',
        
        # ROS Parameters
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'laser',  # Change to your robot's base frame
        'scan_topic': '/scan',
        
        # Map parameters
        'mode': 'mapping',
        'map_file_name': '',
        'map_start_pose': [0.0, 0.0, 0.0],
        'map_start_at_dock': True,
        
        # Sync and Processing
        'debug_logging': False,
        'throttle_scans': 1,
        'transform_publish_period': 0.02,
        'map_update_interval': 5.0,
        'resolution': 0.05,
        'max_laser_range': 12.0,
        'minimum_time_interval': 0.5,
        'transform_timeout': 0.2,
        'tf_buffer_duration': 30.0,
        
        # General Parameters
        'use_scan_matching': True,
        'use_scan_barycenter': True,
        'minimum_travel_distance': 0.5,
        'minimum_travel_heading': 0.5,
        'scan_buffer_size': 10,
        'scan_buffer_maximum_scan_distance': 10.0,
        'link_match_minimum_response_fine': 0.1,
        'link_scan_maximum_distance': 1.5,
        'loop_search_maximum_distance': 3.0,
        'do_loop_closing': True,
        'loop_match_minimum_chain_size': 10,
        'loop_match_maximum_variance_coarse': 3.0,
        'loop_match_minimum_response_coarse': 0.35,
        'loop_match_minimum_response_fine': 0.45,
        
        # Correlation Parameters
        'correlation_search_space_dimension': 0.5,
        'correlation_search_space_resolution': 0.01,
        'correlation_search_space_smear_deviation': 0.1,
        
        # Loop Closure Parameters
        'loop_search_space_dimension': 8.0,
        'loop_search_space_resolution': 0.05,
        'loop_search_space_smear_deviation': 0.03,
        
        # Scan Matcher Parameters
        'distance_variance_penalty': 0.5,
        'angle_variance_penalty': 1.0,
        'fine_search_angle_offset': 0.00349,
        'coarse_search_angle_offset': 0.349,
        'coarse_angle_resolution': 0.0349,
        'minimum_angle_penalty': 0.9,
        'minimum_distance_penalty': 0.5,
        'use_response_expansion': True,
    }
    
    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )
    
    # Static transform from odom to laser (if needed)
    # Adjust this based on your robot configuration
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_odom_laser',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'laser'],
    # )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        slam_toolbox_node,
        # static_tf_node,
    ])
