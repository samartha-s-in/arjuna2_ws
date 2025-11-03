#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped
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

class MultiPointTerminal(Node):
    def __init__(self):
        super().__init__('multi_point_terminal')
        
        # Position and orientation
        self.position_ = Point()
        self.yaw_ = 0.0
        self.state_ = 0
        
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
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Multi-Point Navigation (Terminal) Started")
        self.get_logger().info("===========================================")
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
        
    def navigate_to_goal(self, goal_x, goal_y):
        """Navigate to a single goal point"""
        desired_pose = Point()
        desired_pose.x = goal_x
        desired_pose.y = goal_y
        
        # Reset avoidance state
        reset_avoidance_state()
        
        # Initialize state
        self.state_ = 0
        self.last_position_.x = self.position_.x
        self.last_position_.y = self.position_.y
        self.stuck_time_ = 0.0
        
        self.get_logger().info("-------------------------")
        self.get_logger().info(f"Goal: ({goal_x:.2f}, {goal_y:.2f})")
        self.get_logger().info("-------------------------")
        
        rate = self.create_rate(10)
        
        while rclpy.ok():
            # Check for stuck
            self.check_if_stuck()
            
            # Check for obstacles - priority
            if self.regions and is_obstacle_detected(self.regions):
                obstacle_cmd = avoid_obstacle(self.regions)
                if obstacle_cmd:
                    self.cmd_pub.publish(obstacle_cmd)
                    rate.sleep()
                    continue
            
            # Execute navigation
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
                self.get_logger().info("Waypoint reached")
                return True
            else:
                self.get_logger().error(f"Unknown state: {self.state_}")
                stop()
                return False
            
            # Display progress
            if int(self.get_clock().now().seconds_nanoseconds()[0]) % 3 == 0:
                distance_to_goal = math.sqrt(
                    (desired_pose.y - self.position_.y)**2 +
                    (desired_pose.x - self.position_.x)**2
                )
                self.get_logger().info(f"Distance: {distance_to_goal:.2f}m")
                
            rate.sleep()
        
        return False
    
    def get_waypoints(self):
        """Collect waypoints from user"""
        waypoints = []
        print("\n===========================================")
        print("MULTI-POINT WAYPOINT COLLECTION")
        print("===========================================")
        print("Enter coordinates for each waypoint.")
        print("Type 'done' when finished.")
        print("")
        
        waypoint_num = 1
        while True:
            print(f"Waypoint #{waypoint_num}:")
            user_input = input("  Enter X,Y coordinates (or 'done'): ")
            
            if user_input.lower() == 'done':
                if len(waypoints) == 0:
                    print("  No waypoints added. Add at least one.")
                    continue
                else:
                    break
            
            try:
                coords = user_input.split(',')
                if len(coords) != 2:
                    print("  Invalid format. Use: X,Y (example: 1.5,2.3)")
                    continue
                    
                x = float(coords[0].strip())
                y = float(coords[1].strip())
                
                waypoints.append((x, y))
                print(f"  ✓ Waypoint #{waypoint_num} added: ({x:.2f}, {y:.2f})")
                waypoint_num += 1
                
            except ValueError:
                print("  Invalid input. Enter numbers or 'done'.")
                continue
        
        print("\n===========================================")
        print("WAYPOINT COLLECTION COMPLETE")
        print("===========================================")
        print(f"Total waypoints: {len(waypoints)}")
        for i, (x, y) in enumerate(waypoints):
            print(f"  #{i+1}: ({x:.2f}, {y:.2f})")
        print("")
        
        return waypoints
    
    def execute_waypoints(self, waypoints):
        """Navigate through all waypoints"""
        print("===========================================")
        print("STARTING MULTI-POINT NAVIGATION")
        print("===========================================")
        print(f"Total waypoints: {len(waypoints)}")
        print("")
        
        for i, (x, y) in enumerate(waypoints):
            print(f"\n>>> Waypoint {i+1}/{len(waypoints)}: ({x:.2f}, {y:.2f})")
            
            success = self.navigate_to_goal(x, y)
            
            if not success:
                print(f"\n✗ Navigation failed at waypoint #{i+1}")
                return False
            
            print(f"✓ Waypoint #{i+1} reached successfully")
            
            # Pause between waypoints
            if i < len(waypoints) - 1:
                sleep(1)
        
        print("\n===========================================")
        print("ALL WAYPOINTS COMPLETED SUCCESSFULLY")
        print("===========================================")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = MultiPointTerminal()
    
    # Wait for initial data
    sleep(1)
    
    try:
        while rclpy.ok():
            # Get waypoints
            waypoints = node.get_waypoints()
            
            # Execute navigation
            node.execute_waypoints(waypoints)
            
            # Ask to continue
            print("")
            cont = input("Plan another multi-point route? (y/n): ")
            if cont.lower() != 'y':
                print("\nExiting multi-point navigation.")
                break
                
    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
