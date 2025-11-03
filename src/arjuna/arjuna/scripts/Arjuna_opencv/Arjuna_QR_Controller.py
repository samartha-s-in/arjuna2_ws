#!/usr/bin/env python3
"""
QR Controller - Receives QR position and controls robot movement
Smoothly aligns robot with QR code center, then approaches
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class QRController(Node):
    def __init__(self):
        super().__init__('qr_controller')
        
        # Declare parameters for velocities
        self.declare_parameter('alignment_angular_velocity', 0.2)
        self.declare_parameter('approach_linear_velocity', 0.1)
        self.declare_parameter('target_distance', 30.0)  # cm
        self.declare_parameter('distance_tolerance', 5.0)  # cm
        
        # Get parameters
        self.align_angular_vel = self.get_parameter('alignment_angular_velocity').value
        self.approach_linear_vel = self.get_parameter('approach_linear_velocity').value
        self.target_distance = self.get_parameter('target_distance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        # State variables
        self.qr_position = "none"
        self.qr_distance = 0.0
        self.qr_data = ""
        
        # LIDAR regions
        self.regions = {}
        
        # Control state
        self.is_aligned = False
        self.is_at_target = False
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.pos_sub = self.create_subscription(
            String,
            '/qr_position',
            self.position_callback,
            10
        )
        
        self.dist_sub = self.create_subscription(
            Float32,
            '/qr_distance',
            self.distance_callback,
            10
        )
        
        self.data_sub = self.create_subscription(
            String,
            '/qr_data',
            self.data_callback,
            10
        )
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Timer for control loop (10Hz for smooth control)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("===========================================")
        self.get_logger().info("QR Controller Node Started")
        self.get_logger().info("===========================================")
        self.get_logger().info(f"Alignment angular velocity: {self.align_angular_vel} rad/s")
        self.get_logger().info(f"Approach linear velocity: {self.approach_linear_vel} m/s")
        self.get_logger().info(f"Target distance: {self.target_distance} cm")
        
    def position_callback(self, msg):
        """Receive QR position"""
        self.qr_position = msg.data
        
    def distance_callback(self, msg):
        """Receive QR distance"""
        self.qr_distance = msg.data
        
    def data_callback(self, msg):
        """Receive QR data"""
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
        
    def check_obstacles(self):
        """Check if obstacles block the path"""
        if not self.regions:
            return False
            
        # Check if path is blocked
        return (self.regions['front_L'] < 0.30 or 
                self.regions['front_R'] < 0.30)
        
    def control_loop(self):
        """Main control loop - runs at 10Hz"""
        twist = Twist()
        
        # Check if QR code detected
        if self.qr_data != "Arjuna":
            # No QR code - stop
            self.stop()
            self.is_aligned = False
            self.is_at_target = False
            return
        
        # Check for obstacles
        if self.check_obstacles():
            self.stop()
            self.get_logger().warn("Obstacle detected! Stopping.")
            return
        
        # State machine for QR tracking
        if self.qr_position == "left":
            # QR on left - turn left slowly
            self.is_aligned = False
            twist.angular.z = self.align_angular_vel
            twist.linear.x = 0.0
            self.get_logger().info("Aligning: Turning LEFT")
            
        elif self.qr_position == "right":
            # QR on right - turn right slowly
            self.is_aligned = False
            twist.angular.z = -self.align_angular_vel
            twist.linear.x = 0.0
            self.get_logger().info("Aligning: Turning RIGHT")
            
        elif self.qr_position == "center":
            # QR centered - check distance
            self.is_aligned = True
            
            distance_error = self.qr_distance - self.target_distance
            
            if abs(distance_error) <= self.distance_tolerance:
                # At target distance
                self.stop()
                self.is_at_target = True
                self.get_logger().info("TARGET REACHED - QR aligned and at distance")
                return
                
            elif distance_error > self.distance_tolerance:
                # Too far - move forward slowly
                twist.linear.x = self.approach_linear_vel
                twist.angular.z = 0.0
                self.get_logger().info(f"Approaching: {self.qr_distance:.1f}cm (target: {self.target_distance:.1f}cm)")
                
            else:
                # Too close - move backward slowly
                twist.linear.x = -self.approach_linear_vel * 0.5
                twist.angular.z = 0.0
                self.get_logger().info(f"Too close: {self.qr_distance:.1f}cm - backing up")
        
        else:
            # No QR detected
            self.stop()
            self.is_aligned = False
            self.is_at_target = False
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
    node = QRController()
    
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
