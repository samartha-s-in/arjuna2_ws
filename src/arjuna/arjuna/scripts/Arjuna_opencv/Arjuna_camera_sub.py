#!/usr/bin/env python3
"""
Camera Subscriber - Displays camera feed and depth information
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class CustomCvBridge:
    """Custom cv_bridge replacement for Python 3 compatibility"""
    
    def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
        """Convert a ROS image message to an OpenCV image"""
        if img_msg.encoding == "bgr8":
            cv_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                img_msg.height, img_msg.width, 3
            )
        elif img_msg.encoding == "mono8":
            cv_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                img_msg.height, img_msg.width
            )
        else:
            raise ValueError(f"Unsupported encoding: {img_msg.encoding}")
        
        return cv_image

class CameraDisplay(Node):
    def __init__(self):
        super().__init__('camera_display')
        
        # Custom CV bridge
        self.bridge = CustomCvBridge()
        
        # Variables
        self.rgb_frame = None
        self.depth_value = 0.0
        self.last_valid_depth = 0.0
        self.frame_count = 0
        self.start_time = self.get_clock().now()
        self.fps = 0.0
        self.text_color = (0, 255, 0)  # Default green
        self.depth_text = "Depth: -- cm"
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/frames',
            self.rgb_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Float32,
            '/oak/depth_value',
            self.depth_callback,
            10
        )
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Camera Display Node Started")
        self.get_logger().info("===========================================")
        self.get_logger().info("Subscribing to: /frames, /oak/depth_value")
        self.get_logger().info("Press 'q' in image window to quit")
        
    def rgb_callback(self, msg):
        """Process RGB image"""
        try:
            # Convert ROS Image to OpenCV image
            self.rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Update FPS
            self.frame_count += 1
            current_time = self.get_clock().now()
            elapsed = (current_time - self.start_time).nanoseconds / 1e9
            
            if elapsed > 1.0:
                self.fps = self.frame_count / elapsed
                self.frame_count = 0
                self.start_time = current_time
            
            # Draw info on image
            self.draw_info()
            
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {e}")
    
    def depth_callback(self, msg):
        """Process depth value"""
        self.depth_value = msg.data
        
        # Update text and color based on depth
        if self.depth_value < 0:
            self.depth_text = "Depth: Too close"
            self.text_color = (0, 0, 255)  # Red
        else:
            self.depth_text = f"Depth: {self.depth_value:.1f} cm"
            
            # Color based on distance
            if self.depth_value < 30:
                self.text_color = (0, 0, 255)  # Red - close
            elif self.depth_value < 100:
                self.text_color = (0, 255, 0)  # Green - medium
            else:
                self.text_color = (255, 255, 0)  # Cyan - far
            
            # Update last valid depth
            self.last_valid_depth = self.depth_value
    
    def draw_info(self):
        """Draw information overlay on frame"""
        if self.rgb_frame is None:
            return
        
        # Create a copy
        display_frame = self.rgb_frame.copy()
        
        # Create background rectangle for text
        cv2.rectangle(display_frame, (10, 10), (300, 80), (0, 0, 0), -1)
        
        # Display depth text
        cv2.putText(display_frame, self.depth_text, (15, 35), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.text_color, 2)
        
        # Display FPS
        cv2.putText(display_frame, f"FPS: {self.fps:.1f}", (15, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Draw crosshair at center
        h, w = display_frame.shape[:2]
        center_x, center_y = w // 2, h // 2
        cv2.line(display_frame, (center_x-20, center_y), (center_x+20, center_y), 
                (0, 255, 255), 2)
        cv2.line(display_frame, (center_x, center_y-20), (center_x, center_y+20), 
                (0, 255, 255), 2)
        cv2.circle(display_frame, (center_x, center_y), 5, (0, 255, 255), -1)
        
        # Show frame
        cv2.imshow("OAK Camera Display", display_frame)
        
        # Check for quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Quit signal received")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraDisplay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
