#!/usr/bin/env python3
"""
QR Docking Commands Publisher
Subscribes to turn commands from QR recognition and publishes velocities for docking
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class QRDockingCommands(Node):
    def __init__(self):
        super().__init__('qr_docking_commands')
        
        # Turn command flags
        self.turn_right = 0
        self.turn_left = 0
        self.straight = 0
        
        # LIDAR regions
        self.regions = {}
        
        # Velocities for docking (slower than normal navigation)
        self.linear_velocity = 0.12
        self.angular_velocity = 0.5
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.right_sub = self.create_subscription(
            Int8,
            'turn_right',
            self.turn_right_callback,
            10
        )
        
        self.left_sub = self.create_subscription(
            Int8,
            'turn_left',
            self.turn_left_callback,
            10
        )
        
        self.straight_sub = self.create_subscription(
            Int8,
            'straight',
            self.straight_callback,
            10
        )
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Timer for publishing commands (10Hz for smooth control)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info("===========================================")
        self.get_logger().info("QR Docking Commands Node Started")
        self.get_logger().info("===========================================")
        self.get_logger().info("Subscribing to: turn_right, turn_left, straight, /scan")
        self.get_logger().info("Publishing to: /cmd_vel")
        self.get_logger().info(f"Docking linear velocity: {self.linear_velocity} m/s")
        self.get_logger().info(f"Docking angular velocity: {self.angular_velocity} rad/s")
        
    def turn_right_callback(self, msg):
        """Callback for turn right command"""
        self.turn_right = msg.data
        if msg.data == 1:
            self.get_logger().debug("Received: Turn RIGHT")
        
    def turn_left_callback(self, msg):
        """Callback for turn left command"""
        self.turn_left = msg.data
        if msg.data == 1:
            self.get_logger().debug("Received: Turn LEFT")
        
    def straight_callback(self, msg):
        """Callback for straight command"""
        self.straight = msg.data
        if msg.data == 1:
            self.get_logger().debug("Received: Move STRAIGHT")
        
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
        
    def check_docking_complete(self):
        """Check if docking is complete (obstacles on 3-4 sides)"""
        if not self.regions:
            return False
        
        obstacle_threshold = 0.25  # meters - very close for docking
        
        obstacles = []
        if self.regions['front_L'] < obstacle_threshold:
            obstacles.append('front_L')
        if self.regions['front_R'] < obstacle_threshold:
            obstacles.append('front_R')
        if self.regions['fleft'] < obstacle_threshold:
            obstacles.append('fleft')
        if self.regions['fright'] < obstacle_threshold:
            obstacles.append('fright')
        
        if len(obstacles) >= 3:
            self.get_logger().info(f"DOCKING COMPLETE: {len(obstacles)} sides detected")
            self.get_logger().info(f"Detected obstacles: {obstacles}")
            return True
        
        return False
        
    def publish_velocity(self):
        """Publish velocity based on turn commands and obstacles"""
        twist = Twist()
        
        # Check if docking is complete
        if self.check_docking_complete():
            # Stop - docking complete
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return
        
        # Check if front obstacles too close (stop distance)
        if self.regions:
            if self.regions['front_L'] < 0.20 or self.regions['front_R'] < 0.20:
                # Very close - stop moving forward but allow turning
                if self.turn_right == 1 or self.turn_left == 1:
                    # Allow turning only
                    pass
                else:
                    # Stop completely
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.get_logger().warn("Too close to obstacle - Stopping forward movement")
                    return
        
        # Execute movement based on turn commands
        if self.turn_right == 1 and self.turn_left == 0 and self.straight == 0:
            # Turn right slowly
            twist.linear.x = 0.0
            twist.angular.z = -self.angular_velocity
            self.get_logger().info("Executing: Turn RIGHT")
            
        elif self.turn_left == 1 and self.turn_right == 0 and self.straight == 0:
            # Turn left slowly
            twist.linear.x = 0.0
            twist.angular.z = self.angular_velocity
            self.get_logger().info("Executing: Turn LEFT")
            
        elif self.straight == 1 and self.turn_left == 0 and self.turn_right == 0:
            # Move straight slowly
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0
            self.get_logger().info("Executing: Move STRAIGHT")
            
        else:
            # No clear command - stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().debug("No command - Stopping")
        
        # Publish command
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = QRDockingCommands()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on exit
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_pub.publish(twist)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()