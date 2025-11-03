#!/usr/bin/env python3
"""
QR Recognition for CT (Charge Terminal)
Detects Arjuna QR code and publishes alignment commands for CT docking
Uses different tolerance zones optimized for CT docking
"""

import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
import numpy as np
from pyzbar.pyzbar import decode
from std_msgs.msg import Int8, String

class QRRecognitionCT(Node):
    def __init__(self):
        super().__init__('qr_recognition_ct')
        
        # Publishers for docking control
        self.right_pub = self.create_publisher(Int8, 'turn_right', 10)
        self.left_pub = self.create_publisher(Int8, 'turn_left', 10)
        self.straight_pub = self.create_publisher(Int8, 'straight', 10)
        self.qr_data_pub = self.create_publisher(String, 'qr_data', 10)
        
        # Setup OAK-D camera
        self.pipeline = self.setup_oak_pipeline()
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        self.get_logger().info("===========================================")
        self.get_logger().info("QR Recognition for CT Node Started")
        self.get_logger().info("===========================================")
        self.get_logger().info("Publishing to: turn_right, turn_left, straight, qr_data")
        self.get_logger().info("Optimized for CT docking operations")
        
        # Timer for processing (30Hz)
        self.timer = self.create_timer(0.033, self.process_frame)
        
        # Frame counter for logging
        self.frame_count = 0
        
    def setup_oak_pipeline(self):
        """Setup OAK-D pipeline for RGB camera"""
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
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply adaptive threshold for better contrast
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
            
        self.frame_count += 1
        rows, columns, _ = frame.shape
        
        # CT-specific tolerance zones (tighter for precise docking)
        # These values are optimized for CT docking
        y_tolerance_1 = int(columns / 2.3)  # Left boundary - approximately 278 pixels
        y_tolerance_2 = int(columns / 1.71)  # Right boundary - approximately 374 pixels
        y_center = columns // 2  # Center line - 320 pixels
        
        # Draw tolerance lines on frame
        cv2.line(frame, (y_tolerance_1, 0), (y_tolerance_1, rows), (255, 0, 0), 2)
        cv2.line(frame, (y_tolerance_2, 0), (y_tolerance_2, rows), (255, 0, 0), 2)
        cv2.line(frame, (y_center, 0), (y_center, rows), (0, 255, 0), 1)
        
        # Try direct decoding first
        decoded_image = decode(frame)
        
        # If no QR codes found, try with preprocessing
        if not decoded_image:
            processed_frame = self.preprocess_image(frame)
            decoded_image = decode(processed_frame)
        
        qr_detected = False
        qr_data_string = ""
        
        # Reset command messages
        right_msg = Int8()
        left_msg = Int8()
        straight_msg = Int8()
        right_msg.data = 0
        left_msg.data = 0
        straight_msg.data = 0
        
        # Process detected QR codes
        for barcode in decoded_image:
            string_data = barcode.data.decode("utf-8")
            
            # Only process "Arjuna" QR code
            if string_data == "Arjuna":
                qr_detected = True
                qr_data_string = string_data
                
                # Get QR code bounding box
                x, y, w, h = barcode.rect
                
                # Calculate center of QR code
                qr_center_x = x + w // 2
                qr_center_y = y + h // 2
                
                # Draw GREEN rectangle for Arjuna QR code
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                
                # Draw vertical line at QR center
                cv2.line(frame, (qr_center_x, 0), (qr_center_x, rows), (0, 0, 255), 2)
                
                # Draw center point
                cv2.circle(frame, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)
                
                # Determine movement command based on QR position
                if qr_center_x > y_tolerance_2:
                    # QR is on the RIGHT side - turn RIGHT
                    right_msg.data = 1
                    left_msg.data = 0
                    straight_msg.data = 0
                    
                    cv2.putText(frame, "Command: TURN RIGHT", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    
                    if self.frame_count % 10 == 0:  # Log every 10 frames
                        self.get_logger().info(f"QR RIGHT: center_x={qr_center_x}, boundary={y_tolerance_2}")
                    
                elif qr_center_x < y_tolerance_1:
                    # QR is on the LEFT side - turn LEFT
                    left_msg.data = 1
                    right_msg.data = 0
                    straight_msg.data = 0
                    
                    cv2.putText(frame, "Command: TURN LEFT", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    
                    if self.frame_count % 10 == 0:
                        self.get_logger().info(f"QR LEFT: center_x={qr_center_x}, boundary={y_tolerance_1}")
                    
                else:
                    # QR is in CENTER zone - move STRAIGHT
                    straight_msg.data = 1
                    left_msg.data = 0
                    right_msg.data = 0
                    
                    cv2.putText(frame, "Command: MOVE STRAIGHT", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    
                    if self.frame_count % 10 == 0:
                        self.get_logger().info(f"QR CENTERED: center_x={qr_center_x}")
                
                # Display QR info
                cv2.putText(frame, f"QR: {string_data}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(frame, f"Position: ({qr_center_x}, {qr_center_y})", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
                
                # Only process first Arjuna QR code
                break
            else:
                # Non-Arjuna QR code detected - ignore but show info
                cv2.putText(frame, f"QR (ignored): {string_data}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        
        if not qr_detected:
            # No Arjuna QR code detected
            cv2.putText(frame, "NO ARJUNA QR DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            if self.frame_count % 30 == 0:  # Log every second
                self.get_logger().info("Searching for Arjuna QR code...")
        
        # Publish commands
        self.right_pub.publish(right_msg)
        self.left_pub.publish(left_msg)
        self.straight_pub.publish(straight_msg)
        
        # Publish QR data
        qr_msg = String()
        qr_msg.data = qr_data_string
        self.qr_data_pub.publish(qr_msg)
        
        # Draw tolerance zone labels
        cv2.putText(frame, "LEFT", (y_tolerance_1 - 50, rows - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.putText(frame, "CENTER", (y_center - 30, rows - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, "RIGHT", (y_tolerance_2 + 10, rows - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        # Display frame
        cv2.imshow("QR Recognition for CT", frame)
        
        # Check for quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Quit signal received")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = QRRecognitionCT()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down QR Recognition for CT...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
