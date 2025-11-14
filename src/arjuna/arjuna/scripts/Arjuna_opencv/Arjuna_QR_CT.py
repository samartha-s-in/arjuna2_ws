#!/usr/bin/env python3

"""
Arjuna QR Control
Subscribes to QR tracking commands and LIDAR data
Controls robot motors with obstacle avoidance

Company: NEWRRO TECH LLP
Website: www.newrro.in
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

# ==================== PARAMETERS ====================
LINEAR_VELOCITY = 0.5       # Forward speed (m/s)
ANGULAR_VELOCITY = 0.7      # Turn speed (rad/s)
OBSTACLE_THRESHOLD = 0.35   # Stop if obstacle within 35cm

class QRController(Node):
    def __init__(self):
        super().__init__('arjuna_qr_controller')
        
        # State
        self.current_command = "stop"
        self.obstacle_detected = False
        self.regions = {}
        
        # Statistics
        self.command_count = 0
        self.obstacle_count = 0
        self.start_time = time.time()
        
        # Subscribe to QR tracking commands
        self.qr_cmd_sub = self.create_subscription(
            String,
            '/arjuna/qr_tracking/command',
            self.qr_command_callback,
            10)
        
        # Subscribe to LIDAR
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        
        # Publish motor commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        # Control loop timer (20Hz for responsive control)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Stats timer
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ARJUNA QR CONTROLLER - ACTIVE")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Linear velocity: {LINEAR_VELOCITY} m/s")
        self.get_logger().info(f"Angular velocity: {ANGULAR_VELOCITY} rad/s")
        self.get_logger().info(f"Obstacle threshold: {OBSTACLE_THRESHOLD} m")
        self.get_logger().info("Subscribes: /arjuna/qr_tracking/command, /scan")
        self.get_logger().info("Publishes: /cmd_vel")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Waiting for QR tracking commands...")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
    
    def qr_command_callback(self, msg):
        """Receive QR tracking command"""
        new_command = msg.data
        
        if new_command != self.current_command:
            self.current_command = new_command
            self.command_count += 1
            
            if new_command != "stop":
                self.get_logger().info(f"QR Command: {new_command.upper()}")
    
    def laser_callback(self, msg):
        """Process LIDAR data for obstacle detection"""
        num_ranges = len(msg.ranges)
        
        if num_ranges == 0:
            return
        
        # Helper to safely get region min
        def get_region_min(start_fraction, end_fraction):
            start_idx = int(num_ranges * start_fraction)
            end_idx = int(num_ranges * end_fraction)
            if start_idx >= end_idx or end_idx > num_ranges:
                return 10.0
            region_ranges = msg.ranges[start_idx:end_idx]
            if len(region_ranges) == 0:
                return 10.0
            valid_ranges = [r for r in region_ranges if 0.1 < r < 10.0]
            return min(valid_ranges) if valid_ranges else 10.0
        
        # Critical front regions for QR tracking
        self.regions = {
            'front_left':  get_region_min(0.0, 0.167),     # 0-60°
            'front_right': get_region_min(0.833, 1.0),     # 300-360°
            'f_left':      get_region_min(0.167, 0.333),   # 60-120°
            'f_right':     get_region_min(0.667, 0.833)    # 240-300°
        }
        
        # Check if any front obstacle
        self.obstacle_detected = (
            self.regions['front_left'] < OBSTACLE_THRESHOLD or
            self.regions['front_right'] < OBSTACLE_THRESHOLD or
            self.regions['f_left'] < OBSTACLE_THRESHOLD or
            self.regions['f_right'] < OBSTACLE_THRESHOLD
        )
        
        if self.obstacle_detected:
            self.obstacle_count += 1
    
    def control_loop(self):
        """Main control loop - executes QR commands with obstacle avoidance"""
        cmd = Twist()
        
        # PRIORITY 1: Obstacle safety (overrides QR commands)
        if self.obstacle_detected:
            # Stop if obstacle in any front region
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            if not hasattr(self, '_obstacle_warned'):
                self.get_logger().warn("⚠ OBSTACLE DETECTED - Stopping")
                self._obstacle_warned = True
        
        # PRIORITY 2: Execute QR tracking command
        else:
            # Reset obstacle warning
            if hasattr(self, '_obstacle_warned'):
                delattr(self, '_obstacle_warned')
            
            if self.current_command == "left":
                # Turn left to center QR
                cmd.linear.x = 0.0
                cmd.angular.z = ANGULAR_VELOCITY
            
            elif self.current_command == "center":
                # QR centered - move forward
                cmd.linear.x = LINEAR_VELOCITY
                cmd.angular.z = 0.0
            
            elif self.current_command == "right":
                # Turn right to center QR
                cmd.linear.x = 0.0
                cmd.angular.z = -ANGULAR_VELOCITY
            
            elif self.current_command == "stop":
                # No QR detected - stop
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    def print_stats(self):
        """Print statistics"""
        elapsed = time.time() - self.start_time
        
        self.get_logger().info(
            f"Stats | Commands received: {self.command_count} | "
            f"Obstacles detected: {self.obstacle_count} | "
            f"Current cmd: {self.current_command.upper()} | "
            f"Obstacle: {'YES' if self.obstacle_detected else 'NO'}"
        )
    
    def cleanup(self):
        """Stop robot on shutdown"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info("Robot stopped - QR Controller shutdown")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = QRController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.cleanup()
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()