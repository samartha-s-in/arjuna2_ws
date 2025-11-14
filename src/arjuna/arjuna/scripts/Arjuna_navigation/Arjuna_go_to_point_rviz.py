#!/usr/bin/env python3

"""
Arjuna Autonomous Navigation - Complete Single File
Algorithm:
1. Get initial position → Get goal position
2. Calculate distance (straight line) and heading angle
3. Rotate to face goal
4. Move forward
5. If obstacle → avoid it
6. After avoidance → RECALCULATE from new position (treat as new initial)
7. Repeat steps 2-6 until goal reached

Company: NEWRRO TECH LLP
Website: www.newrro.in
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
import math
import time

# ==================== PARAMETERS ====================
LINEAR_VELOCITY = 0.5           # m/s
ANGULAR_VELOCITY = 0.8          # rad/s
OBSTACLE_DIST_THRESHOLD = 0.35  # 35cm safety distance
GOAL_TOLERANCE = 0.05           # 5cm goal precision
YAW_TOLERANCE = 0.087           # ~5 degrees (pi/36)
UPDATE_RATE = 20.0              # 20Hz - fast updates

class ArjunaNavigator(Node):
    def __init__(self):
        super().__init__('Arjuna_go_to_point_rviz')
        
        # ========== STATE VARIABLES ==========
        self.current_position = Point()
        self.current_yaw = 0.0
        self.goal_position = Point()
        self.has_goal = False
        
        # Navigation calculation results (updated EVERY cycle)
        self.distance_to_goal = 0.0
        self.heading_to_goal = 0.0
        self.angle_error = 0.0
        
        # LIDAR regions
        self.regions = {}
        self.obstacle_detected = False
        
        # State machine: 'idle', 'rotating', 'moving', 'avoiding', 'reached'
        self.nav_state = 'idle'
        
        # ========== ROS 2 INTERFACES ==========
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/robot_pose_ekf/odom_combined',
            self.odom_callback, 
            10)
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_2d',
            self.goal_callback,
            1)
        
        # High-frequency timer for continuous recalculation
        self.timer = self.create_timer(1.0 / UPDATE_RATE, self.navigation_loop)
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ARJUNA AUTONOMOUS NAVIGATION - ACTIVE")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Algorithm: Real-time recalculation with obstacle avoidance")
        self.get_logger().info(f"Update rate: {UPDATE_RATE} Hz")
        self.get_logger().info(f"Linear velocity: {LINEAR_VELOCITY} m/s")
        self.get_logger().info(f"Angular velocity: {ANGULAR_VELOCITY} rad/s")
        self.get_logger().info(f"Goal tolerance: {GOAL_TOLERANCE} m")
        self.get_logger().info(f"Obstacle threshold: {OBSTACLE_DIST_THRESHOLD} m")
        self.get_logger().info("")
        self.get_logger().info("Waiting for goal from RViz [2D Nav Goal]...")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
    
    # ========== CALLBACKS ==========
    
    def odom_callback(self, msg):
        """Update current position and orientation - FAST"""
        # Extract position
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def laser_callback(self, msg):
        """Process LIDAR data into regions - handles any scan size"""
        num_ranges = len(msg.ranges)
        
        if num_ranges == 0:
            return
        
        # Helper function to safely get min distance in a region
        def get_region_min(start_fraction, end_fraction):
            start_idx = int(num_ranges * start_fraction)
            end_idx = int(num_ranges * end_fraction)
            if start_idx >= end_idx or end_idx > num_ranges:
                return 10.0
            region_ranges = msg.ranges[start_idx:end_idx]
            if len(region_ranges) == 0:
                return 10.0
            # Filter out invalid readings (0, inf, nan)
            valid_ranges = [r for r in region_ranges if 0.1 < r < 10.0]
            return min(valid_ranges) if valid_ranges else 10.0
        
        # Divide scan into 6 regions (each ~60 degrees)
        self.regions = {
            'front_left':  get_region_min(0.0, 0.167),    # 0-60°
            'left':        get_region_min(0.167, 0.333),  # 60-120°
            'back_left':   get_region_min(0.333, 0.5),    # 120-180°
            'back_right':  get_region_min(0.5, 0.667),    # 180-240°
            'right':       get_region_min(0.667, 0.833),  # 240-300°
            'front_right': get_region_min(0.833, 1.0)     # 300-360°
        }
        
        # Check if obstacle in path (front regions only)
        self.obstacle_detected = (
            self.regions['front_left'] < OBSTACLE_DIST_THRESHOLD or
            self.regions['front_right'] < OBSTACLE_DIST_THRESHOLD
        )
    
    def goal_callback(self, msg):
        """Receive new goal from RViz"""
        self.goal_position.x = msg.pose.position.x
        self.goal_position.y = msg.pose.position.y
        
        self.has_goal = True
        self.nav_state = 'rotating'
        
        # Calculate initial values
        self.calculate_navigation_params()
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("NEW GOAL RECEIVED")
        self.get_logger().info(f"Current position: ({self.current_position.x:.3f}, {self.current_position.y:.3f})")
        self.get_logger().info(f"Goal position: ({self.goal_position.x:.3f}, {self.goal_position.y:.3f})")
        self.get_logger().info(f"Distance to goal: {self.distance_to_goal:.3f} m")
        self.get_logger().info(f"Heading required: {math.degrees(self.heading_to_goal):.1f}°")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
    
    # ========== NAVIGATION CALCULATIONS ==========
    
    def calculate_navigation_params(self):
        """
        CORE ALGORITHM: Calculate distance and heading from current position to goal
        This runs EVERY cycle - treating current position as new "initial position"
        """
        # Calculate straight-line distance (Pythagorean theorem)
        dx = self.goal_position.x - self.current_position.x
        dy = self.goal_position.y - self.current_position.y
        self.distance_to_goal = math.sqrt(dx * dx + dy * dy)
        
        # Calculate heading angle to goal (atan2 gives correct quadrant)
        self.heading_to_goal = math.atan2(dy, dx)
        
        # Calculate angle error (how much we need to rotate)
        self.angle_error = self.normalize_angle(self.heading_to_goal - self.current_yaw)
    
    def normalize_angle(self, angle):
        """Keep angle between -π and π"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    # ========== MOTION COMMANDS ==========
    
    def stop_robot(self):
        """Stop all motion"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
    
    def rotate_to_goal(self):
        """Rotate to face goal with proportional control"""
        cmd = Twist()
        
        # Proportional control for smooth rotation
        kp_angular = 1.5  # Proportional gain
        cmd.angular.z = kp_angular * self.angle_error
        
        # Clamp to max velocity
        if cmd.angular.z > ANGULAR_VELOCITY:
            cmd.angular.z = ANGULAR_VELOCITY
        elif cmd.angular.z < -ANGULAR_VELOCITY:
            cmd.angular.z = -ANGULAR_VELOCITY
        
        self.cmd_pub.publish(cmd)
    
    def move_to_goal(self):
        """Move forward toward goal with minor heading correction"""
        cmd = Twist()
        
        # Move forward
        cmd.linear.x = LINEAR_VELOCITY
        
        # Small proportional correction to maintain heading
        kp_angular = 0.8
        cmd.angular.z = kp_angular * self.angle_error
        
        # Clamp angular correction
        if cmd.angular.z > ANGULAR_VELOCITY * 0.3:
            cmd.angular.z = ANGULAR_VELOCITY * 0.3
        elif cmd.angular.z < -ANGULAR_VELOCITY * 0.3:
            cmd.angular.z = -ANGULAR_VELOCITY * 0.3
        
        self.cmd_pub.publish(cmd)
    
    def avoid_obstacle(self):
        """Reactive obstacle avoidance"""
        cmd = Twist()
        
        # Analyze obstacle configuration
        front_blocked = (
            self.regions['front_left'] < OBSTACLE_DIST_THRESHOLD or 
            self.regions['front_right'] < OBSTACLE_DIST_THRESHOLD
        )
        left_blocked = self.regions['left'] < OBSTACLE_DIST_THRESHOLD
        right_blocked = self.regions['right'] < OBSTACLE_DIST_THRESHOLD
        
        if front_blocked:
            # Front obstacle - decide which way to turn based on free space
            if self.regions['left'] > self.regions['right']:
                # More space on left - turn left
                cmd.linear.x = 0.0
                cmd.angular.z = ANGULAR_VELOCITY
                self.get_logger().info("Obstacle ahead - turning LEFT")
            else:
                # More space on right - turn right
                cmd.linear.x = 0.0
                cmd.angular.z = -ANGULAR_VELOCITY
                self.get_logger().info("Obstacle ahead - turning RIGHT")
        
        elif left_blocked:
            # Left obstacle - adjust right while moving slowly
            cmd.linear.x = LINEAR_VELOCITY * 0.3
            cmd.angular.z = -ANGULAR_VELOCITY * 0.5
            self.get_logger().info("Obstacle on LEFT - adjusting right")
        
        elif right_blocked:
            # Right obstacle - adjust left while moving slowly
            cmd.linear.x = LINEAR_VELOCITY * 0.3
            cmd.angular.z = ANGULAR_VELOCITY * 0.5
            self.get_logger().info("Obstacle on RIGHT - adjusting left")
        
        self.cmd_pub.publish(cmd)
    
    # ========== MAIN NAVIGATION LOOP ==========
    
    def navigation_loop(self):
        """
        Main control loop - runs at UPDATE_RATE Hz
        RECALCULATES navigation parameters EVERY cycle
        """
        if not self.has_goal:
            return
        
        # ===== STEP 1: RECALCULATE from current position =====
        # This is KEY - we treat current position as new "initial position" every cycle
        self.calculate_navigation_params()
        
        # ===== STEP 2: CHECK if goal reached =====
        if self.distance_to_goal <= GOAL_TOLERANCE:
            if self.nav_state != 'reached':
                self.stop_robot()
                self.nav_state = 'reached'
                self.has_goal = False
                
                self.get_logger().info("")
                self.get_logger().info("=" * 60)
                self.get_logger().info("✓ GOAL REACHED!")
                self.get_logger().info(f"Final position: ({self.current_position.x:.3f}, {self.current_position.y:.3f})")
                self.get_logger().info(f"Distance to goal: {self.distance_to_goal:.3f} m")
                self.get_logger().info("Waiting for new goal...")
                self.get_logger().info("=" * 60)
                self.get_logger().info("")
            return
        
        # ===== STEP 3: OBSTACLE AVOIDANCE (highest priority) =====
        if self.obstacle_detected:
            if self.nav_state != 'avoiding':
                self.get_logger().info("⚠ Obstacle detected - entering avoidance mode")
                self.nav_state = 'avoiding'
            
            self.avoid_obstacle()
            return  # Skip normal navigation while avoiding
        
        # ===== STEP 4: NORMAL NAVIGATION =====
        
        # If we just finished avoiding obstacle, go back to rotation check
        if self.nav_state == 'avoiding':
            self.get_logger().info("✓ Obstacle cleared - recalculating path")
            self.nav_state = 'rotating'
        
        # Check if we need to rotate
        if abs(self.angle_error) > YAW_TOLERANCE:
            if self.nav_state != 'rotating':
                self.get_logger().info(f"↻ Rotating to face goal (error: {math.degrees(self.angle_error):.1f}°)")
                self.nav_state = 'rotating'
            
            self.rotate_to_goal()
        
        # If aligned, move forward
        else:
            if self.nav_state != 'moving':
                self.get_logger().info(f"→ Moving to goal ({self.distance_to_goal:.2f}m remaining)")
                self.nav_state = 'moving'
            
            self.move_to_goal()
        
        # ===== STEP 5: Debug output (every 1 second) =====
        if not hasattr(self, 'last_debug_time'):
            self.last_debug_time = time.time()
        
        if time.time() - self.last_debug_time > 1.0:
            self.get_logger().info(
                f"Pos: ({self.current_position.x:.2f}, {self.current_position.y:.2f}) | "
                f"Dist: {self.distance_to_goal:.2f}m | "
                f"Heading: {math.degrees(self.current_yaw):.0f}° | "
                f"Target: {math.degrees(self.heading_to_goal):.0f}° | "
                f"Error: {math.degrees(self.angle_error):.0f}°"
            )
            self.last_debug_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    
    navigator = ArjunaNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("Shutting down navigation...")
        navigator.stop_robot()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()