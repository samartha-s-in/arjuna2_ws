#!/usr/bin/env python3
"""
Voltage to Webpage
Reads battery voltage from Arduino and publishes for web display
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class VoltagePublisher(Node):
    def __init__(self):
        super().__init__('voltage_publisher')
        
        # Serial port configuration
        self.port = '/dev/ttyUSB0'
        self.baudrate = 9600
        
        # Publisher
        self.pub = self.create_publisher(Float32, 'battery_voltage', 10)
        
        # Initialize serial
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to {self.port}")
        except:
            self.get_logger().error(f"Cannot open {self.port}")
            self.ser = None
        
        # Timer
        self.timer = self.create_timer(1.0, self.read_voltage)
        
    def read_voltage(self):
        """Read voltage from serial"""
        if self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    voltage = float(line)
                    msg = Float32()
                    msg.data = voltage
                    self.pub.publish(msg)
                    self.get_logger().info(f"Voltage: {voltage}V")
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = VoltagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()