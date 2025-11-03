#!/usr/bin/env python3
"""
QR Code Detector - Only detects and publishes QR position
Does NOT control robot movement
"""

import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
from pyzbar.pyzbar import decode
from std_msgs.msg import String, Float32

class QRDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        
        # Publishers
        self.qr_position_pub = self.create_publisher(String, '/qr_position', 10)
        self.qr_distance_pub = self.create_publisher(Float32, '/qr_distance', 10)
        self.qr_data_pub = self.create_publisher(String, '/qr_data', 10)
        
        # Setup OAK-D camera
        self.pipeline = self.setup_oak_pipeline()
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        self.get_logger().info("QR Detector Node Started")
        self.get_logger().info("Publishing QR position to /qr_position")
        self.get_logger().info("Publishing QR distance to /qr_distance")
        
        # Timer for processing (30Hz)
        self.timer = self.create_timer(0.033, self.process_frame)
        
    def setup_oak_pipeline(self):
        """Setup OAK-D pipeline"""
        pipeline = dai.Pipeline()
        
        camRgb = pipeline.create(dai.node.ColorCamera)
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(30)
        camRgb.setPreviewSize(640, 480)
        
        camRgb.preview.link(xoutRgb.input)
        
        return pipeline
        
    def preprocess_image(self, frame):
        """Preprocess for better QR detection"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        adaptive_thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY, 11, 2
        )
        return adaptive_thresh
        
    def process_frame(self):
        """Process camera frame for QR detection"""
        in_rgb = self.q_rgb.tryGet()
        
        if in_rgb is None:
            return
            
        frame = in_rgb.getCvFrame()
        
        if frame is None:
            return
            
        rows, columns, _ = frame.shape
        
        # Define center zone (tolerance)
        center_tolerance = 80  # pixels on each side of center
        center_x = columns // 2
        left_boundary = center_x - center_tolerance
        right_boundary = center_x + center_tolerance
        
        # Draw boundaries on frame
        cv2.line(frame, (left_boundary, 0), (left_boundary, 480), (255, 0, 0), 2)
        cv2.line(frame, (right_boundary, 0), (right_boundary, 480), (255, 0, 0), 2)
        cv2.line(frame, (center_x, 0), (center_x, 480), (0, 255, 0), 2)
        
        # Try direct decoding
        decoded_image = decode(frame)
        
        # If no QR codes, try preprocessing
        if not decoded_image:
            processed_frame = self.preprocess_image(frame)
            decoded_image = decode(processed_frame)
        
        qr_detected = False
        qr_position = "none"
        qr_data_str = ""
        qr_distance = 0.0
        
        # Process QR codes
        for barcode in decoded_image:
            string_data = barcode.data.decode("utf-8")
            
            if string_data == "Arjuna":
                qr_detected = True
                qr_data_str = string_data
                
                x, y, w, h = barcode.rect
                qr_center_x = x + w // 2
                qr_center_y = y + h // 2
                
                # Draw green rectangle
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.line(frame, (qr_center_x, 0), (qr_center_x, 480), (0, 0, 255), 2)
                cv2.circle(frame, (qr_center_x, qr_center_y), 5, (0, 0, 255), -1)
                
                # Estimate distance based on QR code size
                # Assuming QR code is approximately 10cm x 10cm
                # Distance (cm) = (focal_length * real_size) / pixel_size
                focal_length = 500  # Approximate for OAK-D
                real_qr_size = 10.0  # cm
                qr_distance = (focal_length * real_qr_size) / w
                
                # Determine position
                if qr_center_x < left_boundary:
                    qr_position = "left"
                    cv2.putText(frame, "QR: LEFT", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                elif qr_center_x > right_boundary:
                    qr_position = "right"
                    cv2.putText(frame, "QR: RIGHT", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                else:
                    qr_position = "center"
                    cv2.putText(frame, "QR: CENTER", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                
                # Display distance
                cv2.putText(frame, f"Distance: {qr_distance:.1f}cm", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                break  # Only process first Arjuna QR code
        
        if not qr_detected:
            cv2.putText(frame, "NO QR DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        
        # Publish results
        pos_msg = String()
        pos_msg.data = qr_position
        self.qr_position_pub.publish(pos_msg)
        
        dist_msg = Float32()
        dist_msg.data = qr_distance
        self.qr_distance_pub.publish(dist_msg)
        
        data_msg = String()
        data_msg.data = qr_data_str
        self.qr_data_pub.publish(data_msg)
        
        # Display frame
        cv2.imshow("QR Detector", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = QRDetector()
    
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
