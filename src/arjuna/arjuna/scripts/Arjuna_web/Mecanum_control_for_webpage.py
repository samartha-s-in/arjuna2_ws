#!/usr/bin/env python3
"""
Mecanum Control for Web
Receives web commands and controls mecanum wheels
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MecanumWebControl(Node):
    def __init__(self):
        super().__init__('mecanum_web_control')
        
        # Subscribe to cmd_vel
        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )
        
        self.get_logger().info("Mecanum Web Control Started")
        
    def cmd_callback(self, msg):
        """Process velocity commands"""
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        
        # Log received command
        self.get_logger().info(
            f"Cmd: x={linear_x:.2f}, y={linear_y:.2f}, z={angular_z:.2f}"
        )
        
        # Calculate mecanum wheel speeds
        # This is a placeholder - actual motor control should be done
        # by your motor_ops package
        fl = linear_x - linear_y - angular_z
        fr = linear_x + linear_y + angular_z
        bl = linear_x + linear_y - angular_z
        br = linear_x - linear_y + angular_z
        
        self.get_logger().debug(
            f"Wheels: FL={fl:.2f}, FR={fr:.2f}, BL={bl:.2f}, BR={br:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MecanumWebControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()