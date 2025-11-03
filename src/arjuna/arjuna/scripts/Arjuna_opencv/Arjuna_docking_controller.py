#!/usr/bin/env python3
"""
Docking Controller - Docks robot at QR code location
Requires obstacles on 3-4 sides for final docking confirmation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # Parameters
        self.declare_parameter('docking_distance', 15.0)  # cm - very close
        self.declare_parameter('alignment_angular_velocity', 0.15)
        self.declare_parameter('approach_linear_velocity', 0.08)
        
        self.docking_distance = self.get_parameter('docking_distance').value
        self.align_angular_vel = self.get_parameter('alignment_angular_velocity').value
        self.approach_linear_vel = self.get_parameter('approach_linear_velocity').value
        
        # State
        self.qr_position = "none"
        self.qr_distance = 0.0
        self.qr_data = ""
        self.regions = {}
        
        self.docking_state = "searching"  # searching, aligning, approaching, docked
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.pos_sub = self.create_subscription(String, '/qr_position', self.position_callback, 10)
        self.dist_sub = self.create_subscription(Float32, '/qr_distance', self.distance_callback, 10)
        self.data_sub = self.create_subscription(String, '/qr_data', self.data_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Docking Controller Node Started")
        self.get_logger().info("===========================================")
        self.get_logger().info(f"Docking distance: {self.docking_distance} cm")
        
    def position_callback(self, msg):
        self.qr_position = msg.data
        
    def distance_callback(self, msg):
        self.qr_distance = msg.data
        
    def data_callback(self, msg):
        self.qr_data = msg.data
        
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
        
    def check_docking_position(self):
        """Check if robot is in docking position (obstacles on 3-4 sides)"""
        if not self.regions:
            return False
        
        obstacle_threshold = 0.35  # meters
        
        obstacles_detected = []
        
        if self.regions['front_L'] < obstacle_threshold:
            obstacles_detected.append('front_L')
        if self.regions['front_R'] < obstacle_threshold:
            obstacles_detected.append('front_R')
        if self.regions['fleft'] < obstacle_threshold:
            obstacles_detected.append('fleft')
        if self.regions['fright'] < obstacle_threshold:
            obstacles_detected.append('fright')
        if self.regions['left'] < obstacle_threshold:
            obstacles_detected.append('left')
        if self.regions['right'] < obstacle_threshold:
            obstacles_detected.append('right')
        
        # Need at least 3 sides with obstacles
        num_obstacles = len(obstacles_detected)
        
        if num_obstacles >= 3:
            self.get_logger().info(f"Docking position confirmed: {num_obstacles} sides blocked")
            self.get_logger().info(f"Blocked sides: {obstacles_detected}")
            return True
        
        return False
        
    def control_loop(self):
        """Main docking control loop"""
        twist = Twist()
        
        # Check if QR code detected
        if self.qr_data != "Arjuna":
            if self.docking_state != "docked":
                self.stop()
                self.docking_state = "searching"
            return
        
        # State machine
        if self.docking_state == "searching":
            # Found QR code
            self.docking_state = "aligning"
            self.get_logger().info("QR code found - starting alignment")
            
        elif self.docking_state == "aligning":
            # Align with QR code
            if self.qr_position == "left":
                twist.angular.z = self.align_angular_vel
                self.get_logger().info("Aligning: Turning LEFT")
                
            elif self.qr_position == "right":
                twist.angular.z = -self.align_angular_vel
                self.get_logger().info("Aligning: Turning RIGHT")
                
            elif self.qr_position == "center":
                # Aligned - start approaching
                self.docking_state = "approaching"
                self.get_logger().info("Aligned - starting approach")
                
        elif self.docking_state == "approaching":
            # Check alignment
            if self.qr_position != "center":
                # Lost alignment
                self.docking_state = "aligning"
                self.get_logger().warn("Lost alignment - realigning")
                return
            
            # Check if at docking distance
            if self.qr_distance <= self.docking_distance:
                # Check if in docking position (surrounded by obstacles)
                if self.check_docking_position():
                    self.stop()
                    self.docking_state = "docked"
                    self.get_logger().info("="*50)
                    self.get_logger().info("DOCKING SUCCESSFUL!")
                    self.get_logger().info("="*50)
                    return
                else:
                    # Close enough but not in docking position
                    self.get_logger().info(f"At distance ({self.qr_distance:.1f}cm) but not in docking position")
                    twist.linear.x = self.approach_linear_vel * 0.5
            else:
                # Approach slowly
                twist.linear.x = self.approach_linear_vel
                self.get_logger().info(f"Approaching: {self.qr_distance:.1f}cm (target: {self.docking_distance:.1f}cm)")
                
        elif self.docking_state == "docked":
            # Already docked - stay stopped
            self.stop()
            return
        
        # Publish command
        self.cmd_pub.publish(twist)
        
    def stop(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()