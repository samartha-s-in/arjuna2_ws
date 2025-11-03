#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class UltrasonicSubscriber(Node):
    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        
        # Subscribers
        self.array_sub = self.create_subscription(
            Float32MultiArray,
            '/ultrasonic_distances',
            self.array_callback,
            10
        )
        self.point_sub = self.create_subscription(
            Point,
            '/ultrasonic_point',
            self.point_callback,
            10
        )
        
        self.get_logger().info("Ultrasonic Subscriber Node Started")
        
    def array_callback(self, msg):
        """Callback for Float32MultiArray messages"""
        if len(msg.data) >= 2:
            sensor1_distance = msg.data[0]
            sensor2_distance = msg.data[1]
            
            print(f"[Array] Sensor 1: {sensor1_distance} cm | Sensor 2: {sensor2_distance} cm")
            
            # Distance-based alerts
            if sensor1_distance < 10:
                print("  -> WARNING: Sensor 1 detects close obstacle!")
            if sensor2_distance < 10:
                print("  -> WARNING: Sensor 2 detects close obstacle!")
                
    def point_callback(self, msg):
        """Callback for Point messages"""
        sensor1_distance = msg.x
        sensor2_distance = msg.y
        
        print(f"[Point] Sensor 1: {sensor1_distance} cm | Sensor 2: {sensor2_distance} cm")
        
        # Calculate average distance
        avg_distance = (sensor1_distance + sensor2_distance) / 2
        print(f"  -> Average distance: {avg_distance} cm")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
