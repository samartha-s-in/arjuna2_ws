#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
from pyzbar.pyzbar import decode
from std_msgs.msg import String

class SimpleQRDetector(Node):
    def __init__(self):
        super().__init__('simple_qr_detector')
        
        # Publisher
        self.qr_pub = self.create_publisher(String, 'qr_data', 1)
        
        # Setup OAK-D
        self.pipeline = self.setup_oak_pipeline()
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        
        self.get_logger().info("Simple QR Detector Started")
        
        # Timer (30Hz)
        self.timer = self.create_timer(1.0/30.0, self.process_frame)
        
    def setup_oak_pipeline(self):
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
        
    def process_frame(self):
        in_rgb = self.q_rgb.tryGet()
        
        if in_rgb is None:
            return
            
        frame = in_rgb.getCvFrame()
        
        # Try direct decoding
        qr_codes = decode(frame)
        
        # If no codes, try preprocessing
        if not qr_codes:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            thresh = cv2.adaptiveThreshold(
                blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                cv2.THRESH_BINARY, 11, 2
            )
            qr_codes = decode(thresh)
        
        qr_detected = False
        
        for qr in qr_codes:
            qr_detected = True
            qr_data = qr.data.decode('utf-8')
            
            x, y, w, h = qr.rect
            
            if qr_data == "Arjuna":
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, f"Data: {qr_data}", (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                center_x = x + w // 2
                center_y = y + h // 2
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # Publish QR data
            msg = String()
            msg.data = qr_data
            self.qr_pub.publish(msg)
            
            self.get_logger().info(f"Detected: {qr_data}")
        
        if not qr_detected:
            msg = String()
            msg.data = ""
            self.qr_pub.publish(msg)
        
        cv2.imshow("QR Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleQRDetector()
    
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