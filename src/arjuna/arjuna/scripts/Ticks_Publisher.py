#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist
import sys
import os

# Add parent directory to path to import STservo_sdk
# STservo_sdk is at arjuna2_ws/src/arjuna/STservo_sdk/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from STservo_sdk import *

Left_velocity = 0
Right_velocity = 0
Left_Motor_Ticks = 0
Right_Motor_Ticks = 0
MOTOR_ACCL = 0

class TicksPublisher(Node):
    def __init__(self):
        super().__init__("Arjuna_Ticks_Publisher")
        
        # Publishers
        self.left_ticks_pub = self.create_publisher(Int64, "left_ticks", 10)
        self.right_ticks_pub = self.create_publisher(Int64, "right_ticks", 10)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.odom_callback, 10)
        
        self.get_logger().info("")
        self.get_logger().info("Publishers   : left_ticks , right_ticks")
        self.get_logger().info("Subscribers  : cmd_vel")
        self.get_logger().info("")
        
        # Create timer for publishing ticks
        self.timer = self.create_timer(0.1, self.publish_ticks)  # 10Hz
        
        # Initialize motor port - adjust device path as needed
        # ENABLE_WHEEL_MODE()  # Uncomment when motor is connected
    
    def odom_callback(self, msg):
        global Right_velocity, Left_velocity
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        wheel_sep = 0.30

        Right_velocity = ((linear_vel * 2) + (angular_vel * wheel_sep)) / 2.0
        Left_velocity = ((linear_vel * 2) - (angular_vel * wheel_sep)) / 2.0

        Right_velocity = int(Right_velocity * 10000)
        Left_velocity = int(Left_velocity * 10000)
    
    def run_robot(self):
        global MOTOR_ACCL, Left_velocity, Right_velocity
        
        # Uncomment when motor functions are available
        # Run_Motor(1, -Left_velocity, MOTOR_ACCL)  # Left_motor
        # Run_Motor(4, -Left_velocity, MOTOR_ACCL)  # Left_motor
        # Run_Motor(2, Right_velocity, MOTOR_ACCL)  # Right_motor 
        # Run_Motor(3, Right_velocity, MOTOR_ACCL)  # Right_motor
        pass
    
    def publish_ticks(self):
        global Left_Motor_Ticks, Right_Motor_Ticks, Left_velocity, Right_velocity
        
        # Calculate ticks - adjust function names based on actual SDK
        # Left_Motor_Ticks = left_tick_count(Left_velocity)
        # Right_Motor_Ticks = right_tick_count(Right_velocity)
        Left_Motor_Ticks = int(Left_velocity)  # Placeholder
        Right_Motor_Ticks = int(Right_velocity)  # Placeholder
        
        left_msg = Int64()
        left_msg.data = Left_Motor_Ticks
        self.left_ticks_pub.publish(left_msg)
        
        right_msg = Int64()
        right_msg.data = Right_Motor_Ticks
        self.right_ticks_pub.publish(right_msg)
        
        self.get_logger().info(f"Left_Ticks : {Left_Motor_Ticks} | Right_Ticks : {Right_Motor_Ticks}")
        self.run_robot()

def main(args=None):
    rclpy.init(args=args)
    
    publisher = TicksPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # portHandler.closePort()  # Uncomment when available
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

