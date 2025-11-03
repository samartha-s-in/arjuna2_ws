#!/usr/bin/env python3
"""
Complete Arjuna CT (Charge Terminal) Docking System
Integrates navigation, QR recognition, and mecanum control for CT docking
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped
from std_msgs.msg import Int8
import math
from time import sleep
import sys
import os

# Add navigation library
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Newrro_NavLib'))
from Newrro_Navigation import fix_yaw, go_straight_ahead, stop, set_cmd_publisher
from Newrro_obs_script import is_obstacle_detected, avoid_obstacle, reset_avoidance_state

class CTDocking(Node):
    def __init__(self):
        super().__init__('ct_docking')
        
        # Navigation parameters
        self.LINEAR_VELOCITY = 0.15
        self.ANGULAR_VELOCITY = 0.4
        
        # Position and orientation
        self.position_ = Point()
        self.yaw_ = 0.0
        self.state_ = 0
        
        # QR tracking commands
        self.turn_right = 0
        self.turn_left = 0
        self.straight = 0
        
        # LIDAR regions
        self.regions = {}
        
        # Docking state
        self.docking_phase = "navigation"  # navigation, qr_docking, docked
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        set_cmd_publisher(self.cmd_pub)
        
        # Subscribers
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot_pose_ekf/odom_combined',
            self.odom_callback,
            10
        )
        self.right_sub = self.create_subscription(Int8, 'turn_right', self.turn_right_callback, 10)
        self.left_sub = self.create_subscription(Int8, 'turn_left', self.turn_left_callback, 10)
        self.straight_sub = self.create_subscription(Int8, 'straight', self.straight_callback, 10)
        
        self.get_logger().info("===========================================")
        self.get_logger().info("CT Docking System Started")
        self.get_logger().info("===========================================")
        
    def laser_callback(self, msg):
        """Process LIDAR data"""
        self.regions = {
            'front_L': min(min(msg.ranges[0:130]), 10.0),
            'fleft': min(min(msg.ranges[131:230]), 10.0),
            'left': min(min(msg.ranges[231:280]), 10.0),
            'right': min(min(msg.ranges[571:620]), 10.0),
            'fright': min(min(msg.ranges[621:720]), 10.0),
            'front_R': min(min(msg.ranges[721:850]), 10.0)
        }
        
    def odom_callback(self, msg):
        """Process odometry"""
        self.position_ = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw_ = math.atan2(siny_cosp, cosy_cosp)
        
    def turn_right_callback(self, msg):
        self.turn_right = msg.data
        
    def turn_left_callback(self, msg):
        self.turn_left = msg.data
        
    def straight_callback(self, msg):
        self.straight = msg.data
        
    def check_docking_complete(self):
        """Check if docking is complete (obstacles on 3-4 sides)"""
        if not self.regions:
            return False
        
        obstacle_threshold = 0.25
        obstacles = 0
        
        if self.regions['front_L'] < obstacle_threshold:
            obstacles += 1
        if self.regions['front_R'] < obstacle_threshold:
            obstacles += 1
        if self.regions['fleft'] < obstacle_threshold:
            obstacles += 1
        if self.regions['fright'] < obstacle_threshold:
            obstacles += 1
        
        return obstacles >= 3
        
    def navigate_to_ct_point(self, x, y):
        """Navigate to CT approach point"""
        desired_pose = Point()
        desired_pose.x = x
        desired_pose.y = y
        
        reset_avoidance_state()
        self.state_ = 0
        
        self.get_logger().info(f"Navigating to CT point: ({x:.2f}, {y:.2f})")
        self.docking_phase = "navigation"
        
        rate = self.create_rate(10)
        
        while rclpy.ok():
            # Check obstacles
            if self.regions and is_obstacle_detected(self.regions):
                obstacle_cmd = avoid_obstacle(self.regions)
                if obstacle_cmd:
                    self.cmd_pub.publish(obstacle_cmd)
                    rate.sleep()
                    continue
            
            # Navigate
            if self.state_ == 0:
                self.yaw_, self.position_, self.state_ = fix_yaw(
                    desired_pose, self.yaw_, self.position_, self.state_
                )
            elif self.state_ == 1:
                self.yaw_, self.position_, self.state_ = go_straight_ahead(
                    desired_pose, self.yaw_, self.position_, self.state_
                )
            elif self.state_ == 2:
                stop()
                self.get_logger().info("Reached CT approach point")
                return True
                
            rate.sleep()
        
        return False
        
    def qr_guided_docking(self):
        """QR-guided final docking"""
        self.get_logger().info("Starting QR-guided docking")
        self.docking_phase = "qr_docking"
        
        rate = self.create_rate(10)
        timeout_counter = 0
        max_timeout = 300  # 30 seconds at 10Hz
        
        while rclpy.ok():
            timeout_counter += 1
            
            # Check if docked
            if self.check_docking_complete():
                stop()
                self.docking_phase = "docked"
                self.get_logger().info("="*50)
                self.get_logger().info("CT DOCKING SUCCESSFUL!")
                self.get_logger().info("="*50)
                return True
            
            # Check timeout
            if timeout_counter > max_timeout:
                self.get_logger().warn("Docking timeout reached")
                stop()
                return False
            
            # QR commands are handled by QR_docking_commands or mecanum_docking nodes
            # This node just monitors completion
            
            rate.sleep()
        
        return False

def main(args=None):
    rclpy.init(args=args)
    node = CTDocking()
    
    sleep(2)
    
    try:
        print("\n===========================================")
        print("ARJUNA CT DOCKING SYSTEM")
        print("===========================================")
        
        # Get CT coordinates
        x_goal = float(input("Enter X coordinate of CT point: "))
        y_goal = float(input("Enter Y coordinate of CT point: "))
        
        print("\nStarting CT docking sequence...")
        print("Make sure the following nodes are running:")
        print("  1. ros2 run arjuna qr_recog_ct")
        print("  2. ros2 run arjuna qr_docking_commands")
        print("     OR")
        print("     ros2 run arjuna mecanum_docking")
        print("")
        
        input("Press Enter when ready...")
        
        # Navigate to CT point
        if node.navigate_to_ct_point(x_goal, y_goal):
            sleep(2)
            # Start QR-guided docking
            node.qr_guided_docking()
            
    except KeyboardInterrupt:
        print("\nDocking interrupted by user")
    except ValueError:
        print("Invalid coordinates entered")
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
