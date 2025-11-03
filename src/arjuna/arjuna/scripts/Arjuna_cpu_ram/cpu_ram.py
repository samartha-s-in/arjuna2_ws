#!/usr/bin/env python3
"""
QR Recognition for Docking
Detects Arjuna QR code and publishes alignment commands for docking
"""

import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
import numpy as np
from pyzbar.pyzbar import decode
from std_msgs.msg import Int8

class QRRecognition(Node):
    def __init__(self):
        super().__init__('qr_recognition')
        
        # Publishers for docking control
        self.right_pub = self.create_publisher(Int8, 'turn_right', 10)
        self.left_pub = self.create_publisher(Int8, 'turn_left', 10)
        self.straight_pub = self.create_publisher(Int8, 'straight', 10)
        
        # Setup OAK-D camera
        self.pipeline = self.setup_oak_pipeline()
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        self.get_logger().info("===========================================")
        self.get_logger().info("QR Recognition for Docking Node Started")
        self.get_logger().info("===========================================")
        self.get_logger().info("Publishing to: turn_right, turn_left, straight")
        
        # Timer for processing (30Hz)
        self.timer = self.create_timer(0.033, self.process_frame)
        
    def setup_oak_pipeline(self):
        """Setup OAK-D pipeline"""
        pipeline = dai.Pipeline()
        
        # RGB Camera
        camRgb = pipeline.create(dai.node.ColorCamera)
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        
        # Camera properties
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(30)
        camRgb.setPreviewSize(640, 480)
        
        # Linking
        camRgb.preview.link(xoutRgb.input)
        
        return pipeline
        
    def preprocess_image(self, frame):
        """Preprocess image for better QR detection"""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        
        # Apply adaptive threshold
        adaptive_thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY, 11, 2
        )
        
        return adaptive_thresh
        
    def process_frame(self):
        """Process camera frame for QR recognition"""
        in_rgb = self.q_rgb.tryGet()
        
        if in_rgb is None:
            return
            
        frame = in_rgb.getCvFrame()
        
        if frame is None:
            self.get_logger().warn("Failed to capture frame")
            return
            
        rows, columns, _ = frame.shape
        
        # Calculate tolerance zones
        y_tolerance_1 = int(columns / 2.4)  # Left boundary
        y_tolerance_2 = int(columns / 1.61)  # Right boundary
        y_center = int(columns / 2.33)
        
        # Try direct decoding first
        decoded_image = decode(frame)
        
        # If no QR codes found, try with preprocessing
        if not decoded_image:
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            # Apply adaptive threshold
            adaptive_thresh = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                cv2.THRESH_BINARY, 11, 2
            )
            # Try decoding again
            decoded_image = decode(adaptive_thresh)
        
        qr_detected = False
        
        # Process detected QR codes
        for barcode in decoded_image:
            qr_detected = True
            string_data = barcode.data.decode("utf-8")
            
            self.get_logger().info(f"Detected QR code: {string_data}")
            
            if string_data == "Arjuna":
                x, y, w, h = barcode.rect
                y_medium = int(x + w / 2)
                
                # Draw GREEN rectangle for Arjuna QR code
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.line(frame, (y_medium, 0), (y_medium, 480), (0, 0, 255), 1)
                
                # Publish movement commands
                right_msg = Int8()
                left_msg = Int8()
                straight_msg = Int8()
                
                if y_medium > y_tolerance_2:
                    # Turn right
                    self.get_logger().info("Command: Turn RIGHT")
                    right_msg.data = 1
                    left_msg.data = 0
                    straight_msg.data = 0
                    
                elif y_medium < y_tolerance_1:
                    # Turn left
                    self.get_logger().info("Command: Turn LEFT")
                    left_msg.data = 1
                    right_msg.data = 0
                    straight_msg.data = 0
                    
                elif y_medium > y_tolerance_1 and y_medium < y_tolerance_2:
                    # Move straight
                    self.get_logger().info("Command: Move STRAIGHT")
                    straight_msg.data = 1
                    left_msg.data = 0
                    right_msg.data = 0
                
                # Publish commands
                self.right_pub.publish(right_msg)
                self.left_pub.publish(left_msg)
                self.straight_pub.publish(straight_msg)
            else:
                # For non-Arjuna QR codes, don't draw any box
                pass
        
        if not qr_detected:
            self.get_logger().info("NO QR RECOGNISED")
            
            # Publish zero commands
            zero_msg = Int8()
            zero_msg.data = 0
            self.right_pub.publish(zero_msg)
            self.left_pub.publish(zero_msg)
            self.straight_pub.publish(zero_msg)
        
        # Draw tolerance lines
        cv2.line(frame, (y_tolerance_2, 0), (y_tolerance_2, 480), (255, 0, 0), 1)
        cv2.line(frame, (y_tolerance_1, 0), (y_tolerance_1, 480), (255, 0, 0), 1)
        
        # Display frame
        cv2.imshow("QR Recognition", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = QRRecognition()
    
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