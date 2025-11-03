#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist
import sys
from STservo_sdk import *
from time import sleep

# Motor controller parameters
BAUDRATE = 115200
DEVICENAME = '/dev/ttyUSB1'
MOTOR_SPEED = 300
MOTOR_ACCL = 0

class ArjunaTicksPublisher(Node):
    def __init__(self):
        super().__init__("Arjuna_Ticks_Publisher")

        # Publishers
        self.left_ticks_pub = self.create_publisher(Int64, "left_ticks", 10)
        self.right_ticks_pub = self.create_publisher(Int64, "right_ticks", 10)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.odom_callback,
            10
        )

        # Global-like class variables
        self.selector_r = 0
        self.selector_l = 0
        self.left_velocity = 0
        self.right_velocity = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.total_left_ticks = 0
        self.total_right_ticks = 0
        self.encoder_maximum = 32000

        # Serial Communication
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = sts(self.portHandler)
        self.open_port()
        self.set_baudrate()
        self.enable_wheel_mode()

        # Timer for the main loop
        self.timer = self.create_timer(0.1, self.main_loop) # 10 Hz rate

    def open_port(self):
        if self.portHandler.openPort():
            self.get_logger().info("Succeeded to open the port")
        else:
            self.get_logger().error("Failed to open the port")
            sys.exit(1)

    def set_baudrate(self):
        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info("Succeeded to change the baudrate")
        else:
            self.get_logger().error("Failed to change the baudrate")
            sys.exit(1)

    def odom_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        wheel_sep = 0.30
        
        self.right_velocity = ((linear_vel * 2) + (angular_vel * wheel_sep)) / (2.0)
        self.left_velocity = ((linear_vel * 2) - (angular_vel * wheel_sep)) / (2.0)
        
        self.right_velocity = int(self.right_velocity * 10000)
        self.left_velocity = int(self.left_velocity * 10000)

    def wheel_mode(self, Motor_ID):
        Result, Error = self.packetHandler.WheelMode(Motor_ID)
        if Result != COMM_SUCCESS:
            self.get_logger().error(f"WheelMode Error: {self.packetHandler.getTxRxResult(Result)}")
        elif Error != 0:
            self.get_logger().error(f"WheelMode Error: {self.packetHandler.getRxPacketError(Error)}")

    def run_motor(self, Motor_ID, Motor_Speed, Motor_Accel):
        Result, Error = self.packetHandler.WriteSpec(Motor_ID, Motor_Speed, Motor_Accel)
        if Result != COMM_SUCCESS:
            self.get_logger().error(f"Run_Motor Error: {self.packetHandler.getTxRxResult(Result)}")
        if Error != 0:
            self.get_logger().error(f"Run_Motor Error: {self.packetHandler.getRxPacketError(Error)}")

    def present_pos(self, Motor_ID):
        Position, Speed, Result, Error = self.packetHandler.ReadPosSpeed(Motor_ID)
        if Result != COMM_SUCCESS:
            self.get_logger().error(f"Present_Pos Error: {self.packetHandler.getTxRxResult(Result)}")
        if Error != 0:
            self.get_logger().error(f"Present_Pos Error: {self.packetHandler.getRxPacketError(Error)}")
        return Position

    def enable_wheel_mode(self):
        self.wheel_mode(1)
        self.wheel_mode(2)
        self.wheel_mode(3)
        self.wheel_mode(4)

    def right_tick_count(self):
        Right_ticks = int(self.present_pos(2) / 2.5)

        if self.right_velocity > 0:
            self.selector_r = 1
        elif self.right_velocity < 0:
            self.selector_r = 2
        
        if self.selector_r == 1 and self.prev_right_ticks < Right_ticks:
            if self.total_right_ticks >= self.encoder_maximum:
                self.total_right_ticks = 0
            else:
                self.total_right_ticks += abs(Right_ticks - self.prev_right_ticks)
        elif self.selector_r == 2 and self.prev_right_ticks > Right_ticks:
            if self.total_right_ticks <= 0:
                self.total_right_ticks = self.encoder_maximum
            else:
                self.total_right_ticks += (Right_ticks - self.prev_right_ticks)
        
        self.prev_right_ticks = Right_ticks
        
        msg = Int64()
        msg.data = self.total_right_ticks
        self.right_ticks_pub.publish(msg)

    def left_tick_count(self):
        Left_ticks = int(self.present_pos(1) / 2.5)

        if self.left_velocity > 0:
            self.selector_l = 1
        elif self.left_velocity < 0:
            self.selector_l = 2
        
        if self.selector_l == 1 and self.prev_left_ticks > Left_ticks:
            if self.total_left_ticks >= self.encoder_maximum:
                self.total_left_ticks = 0
            else:
                self.total_left_ticks += abs(Left_ticks - self.prev_left_ticks)
        elif self.selector_l == 2 and self.prev_left_ticks < Left_ticks:
            if self.total_left_ticks <= 0:
                self.total_left_ticks = self.encoder_maximum
            else:
                self.total_left_ticks -= (Left_ticks - self.prev_left_ticks)
        
        self.prev_left_ticks = Left_ticks
        
        msg = Int64()
        msg.data = self.total_left_ticks
        self.left_ticks_pub.publish(msg)

    def run_robot(self):
        self.run_motor(1, -self.left_velocity, MOTOR_ACCL)
        self.run_motor(4, -self.left_velocity, MOTOR_ACCL)
        self.run_motor(2, self.right_velocity, MOTOR_ACCL)
        self.run_motor(3, self.right_velocity, MOTOR_ACCL)

    def main_loop(self):
        self.left_tick_count()
        self.right_tick_count()
        self.get_logger().info(f"Left_Ticks : {self.total_left_ticks} | Right_Ticks : {self.total_right_ticks}")
        self.run_robot()

def main(args=None):
    rclpy.init(args=args)
    node = ArjunaTicksPublisher()
    
    print("")
    print("Publishers   : left_ticks, right_ticks")
    print("Subscribers  : cmd_vel")
    print("")
    sleep(2)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.portHandler.closePort()

if __name__ == "__main__":
    main()
