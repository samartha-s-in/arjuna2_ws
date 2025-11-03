#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
from STservo_sdk import *
from time import sleep

# Global variables (not recommended in ROS 2, but we'll keep them for direct conversion)
Left_velocity = 0
Right_velocity = 0

BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB1'
MOTOR_ACCL = 0

portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    sys.exit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    sys.exit()

def odom_callback(msg1):
    global Right_velocity, Left_velocity
    linear_vel = msg1.linear.x
    angular_vel = msg1.angular.z
    wheel_sep = 0.30

    Right_velocity = ((linear_vel * 2) + (angular_vel * wheel_sep)) / (2.0)
    Left_velocity = ((linear_vel * 2) - (angular_vel * wheel_sep)) / (2.0)

    Right_velocity = int(Right_velocity * 10000)
    Left_velocity = int(Left_velocity * 10000)

def wheel_mode(Motor_ID):
    packetHandler.WheelMode(Motor_ID)

def Run_Motor(Motor_ID, Motor_Speed, Motor_Accel):
    packetHandler.WriteSpec(Motor_ID, Motor_Speed, Motor_Accel)

def ENABLE_WHEEL_MODE():
    wheel_mode(1)
    wheel_mode(2)
    wheel_mode(3)
    wheel_mode(4)

def Run_Robot():
    global MOTOR_ACCL, Left_velocity, Right_velocity
    print("Running...")
    Run_Motor(1, -Left_velocity, MOTOR_ACCL)
    Run_Motor(4, -Left_velocity, MOTOR_ACCL)
    Run_Motor(2, Right_velocity, MOTOR_ACCL)
    Run_Motor(3, Right_velocity, MOTOR_ACCL)

class ArjunaCmdVelSub(Node):
    def __init__(self):
        super().__init__('Arjuna_cmd_vel_sub')
        self.subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.odom_callback_wrapper,
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback_wrapper(self, msg):
        odom_callback(msg)

    def timer_callback(self):
        self.get_logger().info("Subscriber for cmd_vel")
        Run_Robot()

def main(args=None):
    rclpy.init(args=args)
    ENABLE_WHEEL_MODE()
    node = ArjunaCmdVelSub()

    try:
        print("")
        print("Publishers : None")
        print("Subscribers : cmd_vel")
        print("")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        portHandler.closePort()

if __name__ == "__main__":
    main()
