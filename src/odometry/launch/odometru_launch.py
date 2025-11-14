#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    pkg_share = FindPackageShare('odometry')
    
    # Launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'odometry_params.yaml']),
        description='Path to odometry config file'
    )
    
    # EKF Odometry Publisher Node
    ekf_data_pub_node = Node(
        package='odometry',
        executable='ekf_data_pub',
        name='ekf_odom_pub',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('odom_data_quat', '/odom'),  # Standard ROS odometry topic
        ]
    )
    
    # RViz Data Publisher Node
    rviz_data_pub_node = Node(
        package='odometry',
        executable='rviz_data_pub',
        name='rviz_data_pub',
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        ekf_data_pub_node,
        rviz_data_pub_node
    ])