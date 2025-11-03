#!/usr/bin/env python3
"""
Mecanum Docking Controller
Handles mecanum wheel control specifically for docking operations
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from time import sleep
import sys
import os

# Add STservo SDK to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'STservo_sdk'))

try:
    from STservo_sdk import *
except ImportError:
    print("WARNING: STservo_sdk not found. Motor control will not be available.")
    print("Please ensure STservo_sdk is in the correct location.")
    MOTORS_AVAILABLE = False
else:
    MOTORS_AVAILABLE = True

class MecanumDocking(Node):
    def __init__(self):
        super().__init__('mecanum_docking')
        
        # Motor parameters
        self.BAUDRATE = 115200
        self.DEVICENAME = '/dev/ttyUSB1'
        self.MOTOR_ACCL = 0
        self.docking_speed = 3500  # Slower for precise docking
        
        # Turn commands from QR
        self.turn_right = 0
        self.turn_left = 0
        self.straight = 0
        
        # Initialize motor communication if available
        if MOTORS_AVAILABLE:
            self.portHandler = PortHandler(self.DEVICENAME)
            self.packetHandler = sts(self.portHandler)
            
            if self.portHandler.openPort():
                self.get_logger().info("Succeeded to open motor port")
            else:
                self.get_logger().error("Failed to open motor port")
                MOTORS_AVAILABLE = False
            
            if MOTORS_AVAILABLE and self.portHandler.setBaudRate(self.BAUDRATE):
                self.get_logger().info("Succeeded to change baudrate")
            else:
                self.get_logger().error("Failed to change baudrate")
                MOTORS_AVAILABLE = False
        
        # Subscribers for QR commands
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
        
        # Enable wheel mode for all motors
        if MOTORS_AVAILABLE:
            self.enable_wheel_mode()
        
        # Timer for controlling motors (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Mecanum Docking Controller Started")
        self.get_logger().info("===========================================")
        self.get_logger().info(f"Docking speed: {self.docking_speed}")
        self.get_logger().info(f"Motors available: {MOTORS_AVAILABLE}")
        
    def turn_right_callback(self, msg):
        """Callback for turn right command"""
        self.turn_right = msg.data
        
    def turn_left_callback(self, msg):
        """Callback for turn left command"""
        self.turn_left = msg.data
        
    def straight_callback(self, msg):
        """Callback for straight command"""
        self.straight = msg.data
        
    def wheel_mode(self, motor_id):
        """Set motor to wheel mode"""
        if not MOTORS_AVAILABLE:
            return
            
        result, error = self.packetHandler.WheelMode(motor_id)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            self.get_logger().error(f"Motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
            
    def run_motor(self, motor_id, motor_speed, motor_accel):
        """Run motor at specified speed"""
        if not MOTORS_AVAILABLE:
            return
            
        result, error = self.packetHandler.WriteSpec(motor_id, motor_speed, motor_accel)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
        if error != 0:
            self.get_logger().error(f"Motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
            
    def enable_wheel_mode(self):
        """Enable wheel mode for all 4 motors"""
        if not MOTORS_AVAILABLE:
            return
            
        for motor_id in [1, 2, 3, 4]:
            self.wheel_mode(motor_id)
        self.get_logger().info("All motors set to wheel mode")
        
    def turn_left_mecanum(self):
        """Turn left using mecanum wheels - in-place rotation"""
        if not MOTORS_AVAILABLE:
            self.get_logger().debug("Simulating: Turn LEFT")
            return
            
        # All wheels rotate in same direction for in-place turn
        self.run_motor(1, self.docking_speed, self.MOTOR_ACCL)
        self.run_motor(2, self.docking_speed, self.MOTOR_ACCL)
        self.run_motor(3, -self.docking_speed, self.MOTOR_ACCL)
        self.run_motor(4, -self.docking_speed, self.MOTOR_ACCL)
        self.get_logger().info("Mecanum: Turning LEFT")
        
    def turn_right_mecanum(self):
        """Turn right using mecanum wheels - in-place rotation"""
        if not MOTORS_AVAILABLE:
            self.get_logger().debug("Simulating: Turn RIGHT")
            return
            
        # All wheels rotate in opposite direction for in-place turn
        self.run_motor(1, -self.docking_speed, self.MOTOR_ACCL)
        self.run_motor(2, -self.docking_speed, self.MOTOR_ACCL)
        self.run_motor(3, self.docking_speed, self.MOTOR_ACCL)
        self.run_motor(4, self.docking_speed, self.MOTOR_ACCL)
        self.get_logger().info("Mecanum: Turning RIGHT")
        
    def move_straight_mecanum(self):
        """Move straight forward using mecanum wheels"""
        if not MOTORS_AVAILABLE:
            self.get_logger().debug("Simulating: Move STRAIGHT")
            return
            
        # Differential drive forward
        self.run_motor(1, -self.docking_speed, self.MOTOR_ACCL)
        self.run_motor(4, -self.docking_speed, self.MOTOR_ACCL)
        self.run_motor(2, self.docking_speed, self.MOTOR_ACCL)
        self.run_motor(3, self.docking_speed, self.MOTOR_ACCL)
        self.get_logger().info("Mecanum: Moving STRAIGHT")
        
    def stop_mecanum(self):
        """Stop all motors"""
        if not MOTORS_AVAILABLE:
            return
            
        for motor_id in [1, 2, 3, 4]:
            self.run_motor(motor_id, 0, self.MOTOR_ACCL)
        self.get_logger().info("Mecanum: STOPPED")
        
    def control_loop(self):
        """Main control loop"""
        # Execute mecanum control based on QR commands
        if self.straight == 1 and self.turn_left == 0 and self.turn_right == 0:
            # Move straight
            self.move_straight_mecanum()
            
        elif self.turn_left == 1 and self.turn_right == 0 and self.straight == 0:
            # Turn left
            self.turn_left_mecanum()
            
        elif self.turn_right == 1 and self.turn_left == 0 and self.straight == 0:
            # Turn right
            self.turn_right_mecanum()
            
        else:
            # No clear command or conflicting commands - stop
            self.stop_mecanum()
            
    def cleanup(self):
        """Cleanup on shutdown"""
        self.stop_mecanum()
        if MOTORS_AVAILABLE:
            self.portHandler.closePort()
            self.get_logger().info("Motor port closed")

def main(args=None):
    rclpy.init(args=args)
    node = MecanumDocking()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()