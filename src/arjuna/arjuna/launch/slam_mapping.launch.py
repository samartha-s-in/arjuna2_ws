#!/usr/bin/env python3
"""
SLAM Mapping with IMU Fusion
Wheel odometry + IMU → robot_localization → fused output for better mapping
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    arjuna_slam_share = FindPackageShare('arjuna')
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='LIDAR serial port'
    )
    
    rviz_config = PathJoinSubstitution([
        arjuna_slam_share,
        'rviz',
        'slam.rviz'
    ])
    
    return LaunchDescription([
        serial_port_arg,
        
        # ========== SENSOR LAYER ==========
        
        # Motor Encoder Publisher
        Node(
            package='motor_ops',
            executable='Arjuna_Ticks_Pub',
            name='Arjuna_Ticks_Pub',
            output='screen'
        ),
        
        # Wheel Odometry (NO TF - robot_localization will publish TF)
        Node(
            package='odometry',
            executable='EKF_data_pub',
            name='EKF_data_pub',
            output='screen',
            parameters=[{'publish_tf': False}],  # CRITICAL: Disable TF
            remappings=[
                ('/odom', '/wheel/odom'),  # Rename to avoid conflict
                ('/odom_data_quat', '/wheel/odom_data_quat'),
                ('/odom_data_euler', '/wheel/odom_data_euler')
            ]
        ),
        
        # IMU
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
            remappings=[
                ('/data', '/imu/data'),
                ('/raw', '/imu/raw'),
                ('/mag', '/imu/mag'),
                ('/status', '/imu/status'),
                ('/temp', 'imu/temp'),
            ],
            output='screen'
        ),
        
        # LIDAR
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': 460800,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            output='screen'
        ),
        
        # ========== TRANSFORM LAYER ==========
        
        # Static TF: base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0.09', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        
        # Static TF: base_link → imu
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu'],
            output='screen'
        ),
        
        # ========== FUSION LAYER ==========
        
        # Robot Localization EKF (fuses wheel + IMU)
        # THIS publishes the TF (odom→base_link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_fusion',
            output='screen',
            parameters=[{
                'frequency': 30.0,
                'sensor_timeout': 1.0,
                'two_d_mode': True,
                'publish_tf': True,  # THIS node publishes TF
                
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                
                # Wheel odometry input
                'odom0': '/wheel/odom',
                'odom0_config': [True,  True,  False,    # x, y position
                                False, False, True,     # yaw 
                                True,  True,  False,    # x, y velocity
                                False, False, True,     # yaw velocity
                                False, False, False],   # acceleration
                'odom0_differential': False,
                
                # IMU input
                'imu0': '/imu/data',
                'imu0_config': [False, False, False,    # position
                              False, False, True,      # yaw (improve orientation)
                              False, False, False,     # velocity
                              False, False, True,      # yaw velocity
                              True,  True,  False],    # x,y accel (detect slip)
                'imu0_differential': False,
                'imu0_remove_gravitational_acceleration': True,
                
                'debug': False
            }],
            remappings=[
                ('/odometry/filtered', '/odom')  # Output as standard /odom
            ]
        ),
        
        # ========== SLAM LAYER ==========
        
        # SLAM Toolbox (uses FUSED /odom)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                
                'minimum_travel_distance': 0.01,
                'minimum_travel_heading': 0.01,
                'minimum_time_interval': 0.1,
                
                'transform_publish_period': 0.02,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 30.0,
                
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'map_update_interval': 1.0,
                
                'throttle_scans': 1,
                'stack_size_to_use': 40000000,
                'enable_interactive_mode': True,
                
                'do_loop_closing': True,
                'loop_search_maximum_distance': 3.0,
                'loop_match_minimum_chain_size': 10,
                
                'scan_buffer_size': 10,
                'scan_buffer_maximum_scan_distance': 10.0,
                'link_match_minimum_response_fine': 0.1,
                'link_scan_maximum_distance': 1.5,
                'correlation_search_space_dimension': 0.5,
                'correlation_search_space_resolution': 0.01
            }]
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])