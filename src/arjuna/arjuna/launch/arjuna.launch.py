#!/usr/bin/env python3
"""
Main Arjuna Launch File (ROS 2)
Minimal version - launches only available components

MISSING FROM ORIGINAL:
- odometry package (EKF_data_pub, Rviz_data_pub)
- Arjuna_ticks_pub.py (motor encoders)
- imu_bno055 package
- robot_pose_ekf (use robot_localization in ROS 2)

This launch file includes:
- Static TF transforms
- RPLIDAR
- RViz
- Map Server
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch.conditions

def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('arjuna')
    
    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start RViz visualization'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([pkg_share, 'maps', 'my_map_9.yaml']),
        description='Path to map file'
    )
    
    return LaunchDescription([
        gui_arg,
        map_file_arg,
        
        # ============================================
        # STATIC TF PUBLISHERS
        # ============================================
        
        # base_link -> laser (LIDAR frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0', '0', '0.09', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        # base_link -> imu
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu'],
            output='screen'
        ),
        
        # base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_broadcaster',
            arguments=['0', '0', '0.09', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),
        
        # map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        # ============================================
        # ODOMETRY & IMU (COMMENTED - EXTERNAL PACKAGES)
        # ============================================
        # Uncomment these when you have the packages installed
        
        # EKF Data Publisher (from odometry package)
        Node(
            package='odometry',
            executable='EKF_data_pub',
            name='EKF_data_pub',
            output='screen'
        ),
        
        #Motor Encoder Ticks Publisher (NEEDS TO BE CREATED)
        Node(
            package='arjuna',
            executable='ticks_pub',
            name='arjuna_ticks_pub',
            output='screen'
        ),
        
        #IMU BNO055 (external package)
        Node(
            package='imu_bno055',
            executable='bno055_i2c_node',
            name='imu_node',
            namespace='imu',
            parameters=[{
                'device': '/dev/i2c-0',
                'address': 40,
                'frame_id': 'imu'
            }],
            respawn=True,
            respawn_delay=2.0,
            output='screen'
        ),
        
        #Robot Localization EKF (ROS 2 equivalent of robot_pose_ekf)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])],
            remappings=[
                ('odometry/filtered', 'robot_pose_ekf/odom_combined'),
                ('odom', 'odom_data_quat'),
                ('imu', 'imu/data')
            ],
            output='screen'
        ),
        
        #RViz Data Publisher (from odometry package)
        Node(
            package='odometry',
            executable='Rviz_data_pub',
            name='rviz_data_pub',
            output='screen'
        ),
        
        # ============================================
        # RPLIDAR NODE
        # ============================================
        Node(
            package='rplidar_ros',
            executable='rplidar_node',  # or 'rplidar_composition' depending on version
            name='rplidarNode',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_frequency': 10.0,
                # 'scan_mode': 'Standard'  # May not be supported in ROS 2 version
            }],
            output='screen'
        ),
        
        # ============================================
        # MAP SERVER
        # ============================================
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map_file'),
                'topic_name': 'map',
                'frame_id': 'map'
            }],
            output='screen'
        ),
        
        # Map Server Lifecycle Manager (needed in ROS 2)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{
                'node_names': ['map_server'],
                'autostart': True
            }],
            output='screen'
        ),
        
        # ============================================
        # RVIZ
        # ============================================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'arjuna.rviz'])],
            condition=launch.conditions.IfCondition(LaunchConfiguration('gui')),
            output='screen'
        ),
    ])