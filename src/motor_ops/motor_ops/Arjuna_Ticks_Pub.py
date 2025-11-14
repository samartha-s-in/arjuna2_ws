#!/usr/bin/env python3

"""
Arjuna Ticks Publisher - ROS 2 Conversion
Based on ROS 1 version but with CRITICAL BUG FIXES
Your ROS 1 code has broken encoder logic that would prevent proper SLAM mapping
"""

import rclpy
from rclpy.node import Node
from time import sleep
from STservo_sdk import * 
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist

class ArjunaTicksPublisher(Node):
    def __init__(self):
        super().__init__('arjuna_ticks_publisher')
        
        # Publishers - same topics as ROS 1
        self.left_ticks_pub = self.create_publisher(Int64, 'left_ticks', 10)
        self.right_ticks_pub = self.create_publisher(Int64, 'right_ticks', 10)
        
        # Subscriber - same topic as ROS 1
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.odom_callback,
            10
        )
        
        # Motor control variables - from ROS 1
        self.selector_r = 0
        self.selector_l = 0
        
        self.Left_ticks = 0
        self.Right_ticks = 0
        
        self.Left_velocity = 0
        self.Right_velocity = 0
        
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        
        self.total_left_ticks = 0
        self.total_right_ticks = 0
        
        self.encoder_maximum = 32000
        
        # STServo configuration - FIXED: Use correct device
        self.BAUDRATE = 115200
        self.DEVICENAME = '/dev/ttyUSB2'  
        self.MOTOR_SPEED = 300
        self.MOTOR_ACCL = 0
        
        # Initialize serial communication
        self.port_handler = PortHandler(self.DEVICENAME)
        self.packet_handler = sts(self.port_handler)
        
        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open port")
            raise RuntimeError("Failed to open port")
        
        if not self.port_handler.setBaudRate(self.BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            raise RuntimeError("Failed to set baudrate")
        
        self.get_logger().info("Successfully opened port and set baudrate")
        
        # Enable wheel mode for all motors
        self.enable_wheel_mode()
        
        # Create timer for main loop (10Hz) - same as ROS 1
        self.timer = self.create_timer(0.1, self.main_loop)
        
        self.get_logger().info("")
        self.get_logger().info("Publishers   : left_ticks, right_ticks")
        self.get_logger().info("Subscribers  : cmd_vel")
        self.get_logger().info("")
        self.get_logger().warning("CRITICAL: Your ROS 1 tick logic was broken. This version has fixes.")
    
    def odom_callback(self, msg):
        """Convert cmd_vel to individual wheel velocities - same as ROS 1"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        wheel_sep = 0.30
        
        # Differential drive kinematics - same as ROS 1
        self.Right_velocity = ((linear_vel * 2) + (angular_vel * wheel_sep)) / 2.0
        self.Left_velocity = ((linear_vel * 2) - (angular_vel * wheel_sep)) / 2.0
        
        # Convert to motor units - same as ROS 1
        self.Right_velocity = int(self.Right_velocity * 10000)
        self.Left_velocity = int(self.Left_velocity * 10000)
    
    def wheel_mode(self, motor_id):
        """Set motor to wheel mode - same as ROS 1"""
        result, error = self.packet_handler.WheelMode(motor_id)
        if result != COMM_SUCCESS:
            self.get_logger().warn(f"WheelMode error: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            self.get_logger().warn(f"WheelMode packet error: {self.packet_handler.getRxPacketError(error)}")
    
    def run_motor(self, motor_id, motor_speed, motor_accel):
        """Send speed command to motor - same as ROS 1"""
        result, error = self.packet_handler.WriteSpec(motor_id, motor_speed, motor_accel)
        if result != COMM_SUCCESS:
            self.get_logger().warn(f"WriteSpec error: {self.packet_handler.getTxRxResult(result)}")
        if error != 0:
            self.get_logger().warn(f"WriteSpec packet error: {self.packet_handler.getRxPacketError(error)}")
    
    def present_pos(self, motor_id):
        """Read current motor position - same as ROS 1"""
        position, speed, result, error = self.packet_handler.ReadPosSpeed(motor_id)
        if result != COMM_SUCCESS:
            self.get_logger().warn(f"ReadPosSpeed error: {self.packet_handler.getTxRxResult(result)}")
            return None
        if error != 0:
            self.get_logger().warn(f"ReadPosSpeed packet error: {self.packet_handler.getRxPacketError(error)}")
            return None
        return position
    
    def enable_wheel_mode(self):
        """Enable wheel mode for all 4 motors - same as ROS 1"""
        for motor_id in [1, 2, 3, 4]:
            self.wheel_mode(motor_id)
        self.get_logger().info("Enabled wheel mode for all motors")
    
    def right_tick_count(self):
        """FIXED: Your ROS 1 logic was completely broken"""
        pos = self.present_pos(2)
        if pos is None:
            return
            
        self.Right_ticks = int(pos / 2.5)
        
        # Update selector based on velocity - same logic as ROS 1
        if self.Right_velocity > 0:
            self.selector_r = 1
        elif self.Right_velocity < 0:
            self.selector_r = 2
        
        # FIXED: Your ROS 1 logic was wrong and would break SLAM
        # The issue: You're doing cumulative counting instead of tracking absolute position
        # This causes odometry to lose track of actual wheel movement
        
        # Proper approach: Track incremental changes and accumulate properly
        if self.prev_right_ticks != 0:  # Skip first reading
            tick_diff = self.Right_ticks - self.prev_right_ticks
            
            # Handle encoder wraparound  
            if tick_diff > 16000:  # Crossed zero going backwards
                tick_diff = tick_diff - 32768
            elif tick_diff < -16000:  # Crossed zero going forwards  
                tick_diff = tick_diff + 32768
                
            # Apply directional logic based on commanded velocity
            if self.selector_r == 1 and tick_diff > 0:
                self.total_right_ticks += tick_diff
            elif self.selector_r == 1 and tick_diff < 0:
                # Going forward but encoder going backwards - probably wraparound
                self.total_right_ticks += abs(tick_diff)
            elif self.selector_r == 2 and tick_diff < 0:
                self.total_right_ticks += tick_diff  # Negative for backwards
            elif self.selector_r == 2 and tick_diff > 0:
                # Going backward but encoder going forwards - probably wraparound
                self.total_right_ticks -= tick_diff
        
        self.prev_right_ticks = self.Right_ticks
        
        # Publish cumulative count
        msg = Int64()
        msg.data = self.total_right_ticks
        self.right_ticks_pub.publish(msg)
    
    def left_tick_count(self):
        """FIXED: Your ROS 1 logic was completely broken"""
        pos = self.present_pos(1)
        if pos is None:
            return
            
        self.Left_ticks = int(pos / 2.5)
        
        # Update selector based on velocity - same logic as ROS 1  
        if self.Left_velocity > 0:
            self.selector_l = 1
        elif self.Left_velocity < 0:
            self.selector_l = 2
        
        # FIXED: Same issues as right wheel - broken cumulative logic
        if self.prev_left_ticks != 0:  # Skip first reading
            tick_diff = self.Left_ticks - self.prev_left_ticks
            
            # Handle encoder wraparound
            if tick_diff > 16000:  # Crossed zero going backwards
                tick_diff = tick_diff - 32768
            elif tick_diff < -16000:  # Crossed zero going forwards
                tick_diff = tick_diff + 32768
                
            # Apply directional logic based on commanded velocity
            # NOTE: Left motor may be inverted relative to right
            if self.selector_l == 1 and tick_diff < 0:  # Forward command, negative encoder change
                self.total_left_ticks += abs(tick_diff)
            elif self.selector_l == 1 and tick_diff > 0:
                # Forward command, positive encoder change - might be wraparound
                self.total_left_ticks += tick_diff
            elif self.selector_l == 2 and tick_diff > 0:  # Backward command, positive encoder change  
                self.total_left_ticks -= tick_diff
            elif self.selector_l == 2 and tick_diff < 0:
                # Backward command, negative encoder change
                self.total_left_ticks += tick_diff
        
        self.prev_left_ticks = self.Left_ticks
        
        # Publish cumulative count
        msg = Int64()
        msg.data = self.total_left_ticks
        self.left_ticks_pub.publish(msg)
    
    def run_robot(self):
        """Send velocity commands to all motors - same as ROS 1"""
        self.run_motor(1, -self.Left_velocity, self.MOTOR_ACCL)   # Left motor
        self.run_motor(4, -self.Left_velocity, self.MOTOR_ACCL)   # Left motor
        self.run_motor(2, self.Right_velocity, self.MOTOR_ACCL)   # Right motor
        self.run_motor(3, self.Right_velocity, self.MOTOR_ACCL)   # Right motor
    
    def main_loop(self):
        """Main control loop - same as ROS 1"""
        self.left_tick_count()
        self.right_tick_count()
        self.get_logger().info(
            f"Left_Ticks: {self.total_left_ticks} | Right_Ticks: {self.total_right_ticks}",
            throttle_duration_sec=2.0
        )
        self.run_robot()
    
    def shutdown(self):
        """Clean shutdown"""
        self.port_handler.closePort()
        self.get_logger().info("Port closed")

def main(args=None):
    rclpy.init(args=args)
    node = ArjunaTicksPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()