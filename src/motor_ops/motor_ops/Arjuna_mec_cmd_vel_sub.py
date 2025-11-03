#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
from STservo_sdk import *
from time import sleep

# Global variables (still here for direct conversion, but a class would be better)
FrontLeft_velocity = 0
FrontRight_velocity = 0
RearLeft_velocity = 0
RearRight_velocity = 0

# Motor controller parameters
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB1'
MOTOR_ACCL = 0

# Set up the port and packet handlers for the motor controller
portHandler = PortHandler(DEVICENAME)
packetHandler = sts(portHandler)

# Open the port to communicate with the motor controller
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    sys.exit()

# Set the baudrate for communication
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    sys.exit()

def odom_callback(msg):
    """
    Callback function that converts cmd_vel messages into individual motor velocities.
    """
    global FrontLeft_velocity, FrontRight_velocity, RearLeft_velocity, RearRight_velocity
    linear_vel = msg.linear.x
    strafe_vel = msg.linear.y
    angular_vel = msg.angular.z

    FrontLeft_velocity = linear_vel + strafe_vel + angular_vel
    FrontRight_velocity = linear_vel - strafe_vel - angular_vel
    RearLeft_velocity = linear_vel - strafe_vel + angular_vel
    RearRight_velocity = linear_vel + strafe_vel - angular_vel

    FrontLeft_velocity = int(FrontLeft_velocity * 10000)
    FrontRight_velocity = int(FrontRight_velocity * 10000)
    RearLeft_velocity = int(RearLeft_velocity * 10000)
    RearRight_velocity = int(RearRight_velocity * 10000)

    print(f"Received: linear_vel={linear_vel}, strafe_vel={strafe_vel}, angular_vel={angular_vel}")
    print(f"Calculated Motor Velocities: FL={FrontLeft_velocity}, FR={FrontRight_velocity}, RL={RearLeft_velocity}, RR={RearRight_velocity}")

def wheel_mode(Motor_ID):
    """Switch motor to wheel mode"""
    packetHandler.WheelMode(Motor_ID)

def run_motor(Motor_ID, Motor_Speed, Motor_Accel):
    """Set the motor speed and acceleration"""
    packetHandler.WriteSpec(Motor_ID, Motor_Speed, Motor_Accel)

def enable_wheel_mode():
    """Enable wheel mode for all motors"""
    wheel_mode(1)
    wheel_mode(2)
    wheel_mode(3)
    wheel_mode(4)

def run_robot():
    """
    Run the robot by setting the speed for all four Mecanum motors.
    """
    global MOTOR_ACCL, FrontLeft_velocity, FrontRight_velocity, RearLeft_velocity, RearRight_velocity

    run_motor(1, -FrontLeft_velocity, MOTOR_ACCL)
    run_motor(2, FrontRight_velocity, MOTOR_ACCL)
    run_motor(4, -RearLeft_velocity, MOTOR_ACCL)
    run_motor(3, RearRight_velocity, MOTOR_ACCL)

class MecanumDriverNode(Node):
    def __init__(self):
        super().__init__('Arjuna_mec_cmd_vel_sub')
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
        run_robot()
        
def main(args=None):
    rclpy.init(args=args)
    enable_wheel_mode()
    node = MecanumDriverNode()
    
    print("\n")
    print("Publishers   : None")
    print("Subscribers  : cmd_vel")
    print("\n")
    sleep(2)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        portHandler.closePort()

if __name__ == "__main__":
    main()
