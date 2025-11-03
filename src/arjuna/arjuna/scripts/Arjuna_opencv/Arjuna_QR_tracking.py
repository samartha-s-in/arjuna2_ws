#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
from pyzbar.pyzbar import decode
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

# Motion parameters
LINEAR_VELOCITY = 0.15
ANGULAR_VELOCITY = 0.7

class QRTracking(Node):
    def __init__(self):
        super().__init__('qr_tracking')
        
        # QR tracking variables
        self.turn_right = 0
        self.turn_left = 0
        self.straight = 0
        
        # LIDAR regions
        self.regions = {}
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.right_pub = self.create_publisher(Int8, 'turn_right', 10)
        self.left_pub = self.create_publisher(Int8, 'turn_left', 10)
        self.straight_pub = self.create_publisher(Int8, 'straight', 10)
        
        # Subscriber
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Setup OAK-D camera
        self.pipeline = self.setup_oak_pipeline()
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        self.get_logger().info("QR Tracking Node Started")
        
        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_frame)
        
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
        
    def laser_callback(self, msg):
        """Process LIDAR data"""
        self.regions = {
            'front_L': min(min(msg.ranges[0:130]), 10.0),
            'fleft': min(min(msg.ranges[131:230]), 10.0),
            'left': min(min(msg.ranges[231:280]), 10.0),
            'right': min(min(msg.ranges[571:620]), 10.0),
            'fright': min(min(msg.ranges[621:720]), 10.0),
            'front_R': min(min(msg.ranges[721:850]), 10.0)
        }
        
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
        """Process camera frame for QR tracking"""
        in_rgb = self.q_rgb.tryGet()
        
        if in_rgb is None:
            return
            
        frame = in_rgb.getCvFrame()
        
        if frame is None:
            return
            
        rows, columns, _ = frame.shape
        y_tolerance_1 = int(columns / 2.3)
        y_tolerance_2 = int(columns / 1.71)
        
        # Draw tolerance lines
        cv2.line(frame, (y_tolerance_2, 0), (y_tolerance_2, 480), (255, 0, 0), 1)
        cv2.line(frame, (y_tolerance_1, 0), (y_tolerance_1, 480), (255, 0, 0), 1)
        
        # Try direct decoding
        decoded_image = decode(frame)
        
        # If no QR codes, try preprocessing
        if not decoded_image:
            processed_frame = self.preprocess_image(frame)
            decoded_image = decode(processed_frame)
        
        qr_detected = False
        
        # Reset control flags
        self.turn_right = 0
        self.turn_left = 0
        self.straight = 0
        
        # Process QR codes
        for barcode in decoded_image:
            qr_detected = True
            string_data = barcode.data.decode("utf-8")
            
            if string_data == "Arjuna":
                x, y, w, h = barcode.rect
                y_medium = int(x + w / 2)
                
                # Draw green rectangle
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.line(frame, (y_medium, 0), (y_medium, 480), (0, 0, 255), 1)
                
                # Determine movement
                if y_medium > y_tolerance_2:
                    self.turn_right = 1
                    msg = Int8()
                    msg.data = 1
                    self.right_pub.publish(msg)
                    
                    msg.data = 0
                    self.left_pub.publish(msg)
                    self.straight_pub.publish(msg)
                    
                    cv2.putText(frame, "Turning Right", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                elif y_medium < y_tolerance_1:
                    self.turn_left = 1
                    msg = Int8()
                    msg.data = 1
                    self.left_pub.publish(msg)
                    
                    msg.data = 0
                    self.right_pub.publish(msg)
                    self.straight_pub.publish(msg)
                    
                    cv2.putText(frame, "Turning Left", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                else:
                    self.straight = 1
                    msg = Int8()
                    msg.data = 1
                    self.straight_pub.publish(msg)
                    
                    msg.data = 0
                    self.left_pub.publish(msg)
                    self.right_pub.publish(msg)
                    
                    cv2.putText(frame, "Moving Straight", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if not qr_detected:
            cv2.putText(frame, "NO QR RECOGNISED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            msg = Int8()
            msg.data = 0
            self.right_pub.publish(msg)
            self.left_pub.publish(msg)
            self.straight_pub.publish(msg)
        
        # Execute movement based on LIDAR and QR detection
        if self.regions:
            if self.regions['front_L'] > 0.30 and self.regions['front_R'] > 0.30:
                if self.turn_right == 1:
                    self.turn_right_velo()
                    cv2.putText(frame, "Movement: Right", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                elif self.turn_left == 1:
                    self.turn_left_velo()
                    cv2.putText(frame, "Movement: Left", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                elif self.straight == 1:
                    self.move_straight_velo()
                    cv2.putText(frame, "Movement: Straight", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                else:
                    self.stop()
                    cv2.putText(frame, "Movement: Stop", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                self.stop()
                cv2.putText(frame, "OBSTACLE DETECTED", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Display LIDAR info
        if self.regions:
            cv2.putText(frame, f"Front L: {self.regions['front_L']:.2f}m", 
                       (columns-200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Front R: {self.regions['front_R']:.2f}m",
                       (columns-200, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow("QR Tracking", frame)
        cv2.waitKey(1)
        
    def turn_left_velo(self):
        """Turn left"""
        twist_msg = Twist()
        twist_msg.angular.z = ANGULAR_VELOCITY
        self.cmd_pub.publish(twist_msg)
        
    def turn_right_velo(self):
        """Turn right"""
        twist_msg = Twist()
        twist_msg.angular.z = -ANGULAR_VELOCITY
        self.cmd_pub.publish(twist_msg)
        
    def move_straight_velo(self):
        """Move straight"""
        twist_msg = Twist()
        twist_msg.linear.x = LINEAR_VELOCITY
        twist_msg.angular.z = 0.0
        self.cmd_pub.publish(twist_msg)
        
    def stop(self):
        """Stop robot"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QRTracking()
    
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
