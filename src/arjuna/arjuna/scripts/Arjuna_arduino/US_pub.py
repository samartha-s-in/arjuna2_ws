#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # Publishers
        self.sensor_pub = self.create_publisher(
            Float32MultiArray, 
            '/ultrasonic_distances', 
            10
        )
        self.point_pub = self.create_publisher(
            Point, 
            '/ultrasonic_point', 
            10
        )
        
        # Initialize serial connection
        try:
            self.arduino = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=1
            )
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            return
        
        # Create timer for reading (10Hz)
        self.timer = self.create_timer(0.1, self.read_and_publish)
        
    def read_and_publish(self):
        try:
            # Read from Arduino
            line = self.arduino.readline().decode('utf-8').strip()
            
            if line:
                # Parse JSON data
                data = json.loads(line)
                sensor1_distance = float(data.get('sensor1', 0))
                sensor2_distance = float(data.get('sensor2', 0))
                
                # Create and publish Float32MultiArray
                array_msg = Float32MultiArray()
                array_msg.data = [sensor1_distance, sensor2_distance]
                self.sensor_pub.publish(array_msg)
                
                # Create and publish Point message
                point_msg = Point()
                point_msg.x = sensor1_distance
                point_msg.y = sensor2_distance
                point_msg.z = 0.0
                self.point_pub.publish(point_msg)
                
                self.get_logger().info(
                    f"Published: Sensor1={sensor1_distance}cm, Sensor2={sensor2_distance}cm"
                )
                
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON received: {line}")
        except Exception as e:
            self.get_logger().error(f"Error reading from Arduino: {e}")
    
    def destroy_node(self):
        if hasattr(self, 'arduino'):
            self.arduino.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
