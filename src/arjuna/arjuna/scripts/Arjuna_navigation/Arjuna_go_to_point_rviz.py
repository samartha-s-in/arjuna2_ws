#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped, PoseStamped
import math
from time import sleep
import sys
import os

# Add Newrro_NavLib to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Newrro_NavLib'))
from Newrro_Navigation import (
    fix_yaw, go_straight_ahead, stop, set_cmd_publisher
)
from Newrro_obs_script import (
    is_path_clear, avoid_obstacle, is_obstacle_detected, reset_avoidance_state
)

# Navigation parameters
LINEAR_VELOCITY = 0.5
ANGULAR_VELOCITY = 0.8
RECOVERY_TIMEOUT = 5.0

class GoToPointRViz(Node):
    def __init__(self):
        super().__init__('go_to_point_rviz')
        
        # Position and orientation
        self.position_ = Point()
        self.yaw_ = 0.0
        self.state_ = 0
        
        # Goal management
        self.goal_position_ = Point()
        self.has_new_goal = False
        self.last_goal_x = 0.0
        self.last_goal_y = 0.0
        self.navigating = False
        
        # LIDAR regions
        self.regions = {}
        
        # Stuck detection
        self.last_position_ = Point()
        self.stuck_time_ = 0.0
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        set_cmd_publisher(self.cmd_pub)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot_pose_ekf/odom_combined',
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Timer for navigation loop
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Go To Point (RViz) Node Started")
        self.get_logger().info("===========================================")
        self.get_logger().info("Set goals using '2D Goal Pose' in RViz")
        self.get_logger().info(f"Linear velocity: {LINEAR_VELOCITY} m/s")
        self.get_logger().info(f"Angular velocity: {ANGULAR_VELOCITY} rad/s")
        
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
        """Process odometry data"""
        self.position_ = msg.pose.pose.position
        
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw_ = math.atan2(siny_cosp, cosy_cosp)
        
    def goal_callback(self, msg):
        """Process goal from RViz"""
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y
        
        # Check if this is a new goal (different from last)
        if (abs(new_x - self.last_goal_x) > 0.01 or 
            abs(new_y - self.last_goal_y) > 0.01):
            
            self.goal_position_.x = new_x
            self.goal_position_.y = new_y
            self.last_goal_x = new_x
            self.last_goal_y = new_y
            self.has_new_goal = True
            
            self.get_logger().info("===========================================")
            self.get_logger().info(f"New goal received: ({new_x:.2f}, {new_y:.2f})")
            self.get_logger().info("===========================================")
        
    def check_if_stuck(self):
        """Check if robot is stuck"""
        dist_moved = math.sqrt(
            (self.position_.x - self.last_position_.x)**2 +
            (self.position_.y - self.last_position_.y)**2
        )
        
        if dist_moved < 0.01 and self.state_ == 1:
            self.stuck_time_ += 0.1
            if self.stuck_time_ > RECOVERY_TIMEOUT:
                self.get_logger().warn("Robot stuck! Activating recovery")
                self.state_ = 0
                self.stuck_time_ = 0.0
        else:
            self.stuck_time_ = 0.0
            
        self.last_position_.x = self.position_.x
        self.last_position_.y = self.position_.y
        
    def navigation_loop(self):
        """Main navigation loop called by timer"""
        # Check if we have a new goal
        if self.has_new_goal and not self.navigating:
            # Start new navigation
            self.navigating = True
            self.has_new_goal = False
            self.state_ = 0
            reset_avoidance_state()
            
            self.last_position_.x = self.position_.x
            self.last_position_.y = self.position_.y
            self.stuck_time_ = 0.0
            
            self.get_logger().info("*** NAVIGATION STARTED ***")
        
        # Execute navigation if active
        if self.navigating:
            # Check for stuck
            self.check_if_stuck()
            
            # Check for obstacles - priority
            if self.regions and is_obstacle_detected(self.regions):
                obstacle_cmd = avoid_obstacle(self.regions)
                if obstacle_cmd:
                    self.cmd_pub.publish(obstacle_cmd)
                    return
            
            # Execute navigation based on state
            if self.state_ == 0:
                # Orientation correction
                self.yaw_, self.position_, self.state_ = fix_yaw(
                    self.goal_position_, self.yaw_, self.position_, self.state_
                )
            elif self.state_ == 1:
                # Move straight
                self.yaw_, self.position_, self.state_ = go_straight_ahead(
                    self.goal_position_, self.yaw_, self.position_, self.state_
                )
            elif self.state_ == 2:
                # Goal reached
                stop()
                self.get_logger().info("*** NAVIGATION COMPLETED ***")
                self.get_logger().info("Waiting for new goal from RViz...")
                self.navigating = False
                self.state_ = 0

def main(args=None):
    rclpy.init(args=args)
    node = GoToPointRViz()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
