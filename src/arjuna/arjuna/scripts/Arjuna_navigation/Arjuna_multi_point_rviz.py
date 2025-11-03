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

class MultiPointRViz(Node):
    def __init__(self):
        super().__init__('multi_point_rviz')
        
        # Position and orientation
        self.position_ = Point()
        self.yaw_ = 0.0
        self.state_ = 0
        
        # Waypoint management
        self.waypoints = []
        self.collecting_waypoints = False
        self.executing_waypoints = False
        self.current_waypoint_index = 0
        
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
        
        # Timer for navigation
        self.nav_timer = self.create_timer(0.1, self.navigation_loop)
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Multi-Point Navigation (RViz) Started")
        self.get_logger().info("===========================================")
        self.print_instructions()
        
    def print_instructions(self):
        """Print usage instructions"""
        self.get_logger().info("")
        self.get_logger().info("INSTRUCTIONS:")
        self.get_logger().info("1. Call 'start_collection' service to begin")
        self.get_logger().info("2. Set waypoints using '2D Goal Pose' in RViz")
        self.get_logger().info("3. Call 'execute_waypoints' service to start navigation")
        self.get_logger().info("")
        
        # Create services for waypoint management
        self.start_srv = self.create_service(
            std_srvs.srv.Trigger,
            'start_collection',
            self.start_collection_callback
        )
        self.execute_srv = self.create_service(
            std_srvs.srv.Trigger,
            'execute_waypoints',
            self.execute_waypoints_callback
        )
        self.clear_srv = self.create_service(
            std_srvs.srv.Trigger,
            'clear_waypoints',
            self.clear_waypoints_callback
        )
        
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
        if self.collecting_waypoints:
            waypoint = Point()
            waypoint.x = msg.pose.position.x
            waypoint.y = msg.pose.position.y
            waypoint.z = 0.0
            
            self.waypoints.append(waypoint)
            self.get_logger().info(
                f"Waypoint #{len(self.waypoints)} added: ({waypoint.x:.2f}, {waypoint.y:.2f})"
            )
        
    def start_collection_callback(self, request, response):
        """Start collecting waypoints"""
        self.waypoints = []
        self.collecting_waypoints = True
        self.executing_waypoints = False
        
        response.success = True
        response.message = "Started collecting waypoints. Set goals in RViz."
        self.get_logger().info("=== WAYPOINT COLLECTION STARTED ===")
        return response
        
    def execute_waypoints_callback(self, request, response):
        """Start executing collected waypoints"""
        if len(self.waypoints) == 0:
            response.success = False
            response.message = "No waypoints to execute"
            return response
        
        self.collecting_waypoints = False
        self.executing_waypoints = True
        self.current_waypoint_index = 0
        self.state_ = 0
        reset_avoidance_state()
        
        response.success = True
        response.message = f"Executing {len(self.waypoints)} waypoints"
        self.get_logger().info("=== STARTING WAYPOINT EXECUTION ===")
        self.get_logger().info(f"Total waypoints: {len(self.waypoints)}")
        
        return response
        
    def clear_waypoints_callback(self, request, response):
        """Clear all waypoints"""
        self.waypoints = []
        self.collecting_waypoints = False
        self.executing_waypoints = False
        
        response.success = True
        response.message = "Waypoints cleared"
        self.get_logger().info("Waypoints cleared")
        return response
        
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
        """Main navigation loop"""
        if not self.executing_waypoints:
            return
            
        if self.current_waypoint_index >= len(self.waypoints):
            # All waypoints completed
            stop()
            self.get_logger().info("=== ALL WAYPOINTS COMPLETED ===")
            self.executing_waypoints = False
            return
        
        # Get current waypoint
        current_waypoint = self.waypoints[self.current_waypoint_index]
        
        # Check for stuck
        self.check_if_stuck()
        
        # Check for obstacles
        if self.regions and is_obstacle_detected(self.regions):
            obstacle_cmd = avoid_obstacle(self.regions)
            if obstacle_cmd:
                self.cmd_pub.publish(obstacle_cmd)
                return
        
        # Execute navigation
        if self.state_ == 0:
            self.yaw_, self.position_, self.state_ = fix_yaw(
                current_waypoint, self.yaw_, self.position_, self.state_
            )
        elif self.state_ == 1:
            self.yaw_, self.position_, self.state_ = go_straight_ahead(
                current_waypoint, self.yaw_, self.position_, self.state_
            )
        elif self.state_ == 2:
            # Current waypoint reached
            stop()
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} reached"
            )
            
            # Move to next waypoint
            self.current_waypoint_index += 1
            self.state_ = 0
            reset_avoidance_state()
            
            if self.current_waypoint_index < len(self.waypoints):
                next_wp = self.waypoints[self.current_waypoint_index]
                self.get_logger().info(
                    f"Navigating to waypoint {self.current_waypoint_index + 1}: "
                    f"({next_wp.x:.2f}, {next_wp.y:.2f})"
                )

def main(args=None):
    rclpy.init(args=args)
    
    # Import std_srvs here to avoid issues
    try:
        import std_srvs.srv
    except ImportError:
        print("Installing std_srvs...")
        import subprocess
        subprocess.call(['sudo', 'apt', 'install', '-y', 'ros-humble-std-srvs'])
        import std_srvs.srv
    
    node = MultiPointRViz()
    
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