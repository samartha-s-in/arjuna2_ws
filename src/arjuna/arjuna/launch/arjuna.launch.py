#!/usr/bin/env python3
"""
Arjuna Navigation with IMU Fusion
Wheel odometry + IMU → robot_localization → fused output for better navigation
CRITICAL: EKF_data_pub must have publish_tf parameter support
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess 
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_arjuna = get_package_share_directory('arjuna')
    
    gui_arg = DeclareLaunchArgument(
        'gui', 
        default_value='True',
        description='Whether to start RViz'
    )
    
    map_yaml_file = '/root/arjuna_ros2/arjuna2_ws/src/arjuna/arjuna/maps/my_map.yaml'
    
    # Fix permissions on map files
    os.system(f"chmod -R 755 /root/arjuna_ros2/arjuna2_ws/src/arjuna/arjuna/maps")
    os.system(f"chmod 644 {map_yaml_file}")
    pgm_file = os.path.join(os.path.dirname(map_yaml_file), "my_map.pgm")
    if os.path.exists(pgm_file):
        os.system(f"chmod 644 {pgm_file}")
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=map_yaml_file,
        description='Path to map file'
    )
    
    # ========== TRANSFORM LAYER (Static TFs) ==========
    
    # Map → odom (for localization)
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    base_link_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_broadcaster',
        arguments=['0', '0', '0.09', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0', '0', '0.09', '0', '0', '0', 'base_link', 'laser']
    )
    
    imu_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu']
    )
    
    # ========== MAP SERVER ==========
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'yaml_filename': map_yaml_file,
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': False,
            'autostart': True
        }]
    )
    
    map_server_configure = ExecuteProcess(
        cmd=['sleep', '2', '&&', 'ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        shell=True,
        output='screen'
    )
    
    map_server_activate = ExecuteProcess(
        cmd=['sleep', '3', '&&', 'ros2', 'lifecycle', 'set', '/map_server', 'activate'],
        shell=True,
        output='screen'
    )
    
    map_publisher = ExecuteProcess(
        cmd=['sleep', '5', '&&', 'ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
        shell=True,
        output='screen'
    )
    
    # ========== SENSOR LAYER ==========
    
    # Motor encoder publisher
    arjuna_ticks_pub = Node(
        package='motor_ops',
        executable='Arjuna_Ticks_Pub',
        name='Arjuna_Ticks_Pub'
    )
    
    # Wheel odometry (NO TF - robot_localization will publish TF)
    ekf_data_pub = Node(
        package='odometry',
        executable='EKF_data_pub',
        name='ekf_data_pub',
        parameters=[{'publish_tf': False}],  # CRITICAL: Disable TF
        remappings=[
            ('/odom', '/wheel/odom'),
            ('/odom_data_quat', '/wheel/odom_data_quat'),
            ('/odom_data_euler', '/wheel/odom_data_euler')
        ]
    )
    
    # IMU
    imu_node = Node(
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
    )
    
    # LIDAR
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )
    
    # ========== FUSION LAYER ==========
    
    # Robot Localization EKF (fuses wheel + IMU)
    # THIS publishes the TF (odom→base_link)
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='robot_pose_ekf',
        output='screen',
        parameters=[{
            'frequency': 30.0,
            'sensor_timeout': 1.0,
            'two_d_mode': True,
            'publish_tf': True,  # THIS node publishes TF
            
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            
            # Wheel odometry input
            'odom0': '/wheel/odom',
            'odom0_config': [True,  True,  False,
                            False, False, True,
                            True,  True,  False,
                            False, False, True,
                            False, False, False],
            'odom0_differential': False,
            
            # IMU input
            'imu0': '/imu/data',
            'imu0_config': [False, False, False,
                          False, False, True,
                          False, False, False,
                          False, False, True,
                          True,  True,  False],
            'imu0_differential': False,
            'imu0_remove_gravitational_acceleration': True,
            
            'debug': False
        }],
        remappings=[
            ('odometry/filtered', 'robot_pose_ekf/odom_combined')
        ]
    )
    
    # RViz data publisher
    rviz_data_pub = Node(
        package='odometry',
        executable='Rviz_data_pub',
        name='rviz_data_pub',
        output='screen'
    )
    
    # ========== VISUALIZATION ==========
    
    rviz_config_path = os.path.join(pkg_arjuna, 'rviz', 'arjuna.rviz')
    if not os.path.exists(rviz_config_path):
        alt_rviz_paths = [
            '/root/arjuna_ros2/arjuna2_ws/src/arjuna/rviz/arjuna.rviz',
            '/root/arjuna_ros2/arjuna2_ws/src/arjuna/arjuna/rviz/arjuna.rviz',
            '/root/arjuna_ros2/arjuna2_ws/install/arjuna/share/arjuna/rviz/arjuna.rviz'
        ]
        for path in alt_rviz_paths:
            if os.path.exists(path):
                rviz_config_path = path
                break
    
    if os.path.exists(rviz_config_path):
        os.system(f"chmod 644 {rviz_config_path}")
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        condition=IfCondition(LaunchConfiguration('gui')),
        output='screen'
    )
    
    return LaunchDescription([
        gui_arg,
        map_file_arg,
        # Static TFs
        map_to_odom,
        base_link_broadcaster,
        base_link_to_laser,
        imu_broadcaster,
        # Map server
        map_server_node,
        map_server_configure,
        map_server_activate,
        map_publisher,
        # Sensors
        arjuna_ticks_pub,
        ekf_data_pub,
        imu_node,
        lidar_node,
        # Fusion
        robot_localization_node,  # Publishes TF
        # Utilities
        rviz_data_pub,
        rviz_node
    ])