#!/usr/bin/env python3
"""
Torch Control
Controls robot torch via GPIO
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except:
    GPIO_AVAILABLE = False

class TorchControl(Node):
    def __init__(self):
        super().__init__('torch_control')
        
        self.pin = 17
        
        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin, GPIO.OUT)
            GPIO.output(self.pin, GPIO.LOW)
        
        self.sub = self.create_subscription(
            Bool,
            'torch',
            self.torch_callback,
            10
        )
        
        self.get_logger().info("Torch Control Started")
        
    def torch_callback(self, msg):
        """Control torch"""
        if GPIO_AVAILABLE:
            if msg.data:
                GPIO.output(self.pin, GPIO.HIGH)
                self.get_logger().info("Torch ON")
            else:
                GPIO.output(self.pin, GPIO.LOW)
                self.get_logger().info("Torch OFF")

def main(args=None):
    rclpy.init(args=args)
    node = TorchControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()