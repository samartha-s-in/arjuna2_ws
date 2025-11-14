#!/usr/bin/env python3

"""
Arjuna Camera Publisher - NO cv_bridge
Uses pure Python for maximum compatibility
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import threading
from queue import Queue
import time

# ==================== PARAMETERS ====================
CAMERA_DEVICE = '/dev/video0'
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30
JPEG_QUALITY = 80
QUEUE_SIZE = 10

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('arjuna_camera_publisher')
        
        # Publisher
        self.image_pub = self.create_publisher(
            CompressedImage, 
            '/arjuna/camera/image_raw/compressed', 
            10)
        
        # Stats
        self.frame_count = 0
        self.dropped_frames = 0
        self.start_time = time.time()
        
        # Frame queue
        self.frame_queue = Queue(maxsize=QUEUE_SIZE)
        
        # Open camera
        self.cap = cv2.VideoCapture(CAMERA_DEVICE)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")
            raise RuntimeError("Camera init failed")
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Start capture thread
        self.is_capturing = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        
        # Publishing timer
        timer_period = 1.0 / FPS
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        # Stats timer
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ARJUNA CAMERA PUBLISHER - ACTIVE")
        self.get_logger().info(f"Camera: /dev/video{CAMERA_DEVICE}")
        self.get_logger().info(f"Resolution: {FRAME_WIDTH}x{FRAME_HEIGHT} @ {FPS} FPS")
        self.get_logger().info(f"Topic: /arjuna/camera/image_raw/compressed")
        self.get_logger().info("=" * 60)
    
    def capture_loop(self):
        """Background capture thread"""
        while self.is_capturing:
            ret, frame = self.cap.read()
            
            if not ret:
                self.get_logger().warn("Failed to capture")
                time.sleep(0.01)
                continue
            
            # Drop old frame if queue full
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                    self.dropped_frames += 1
                except:
                    pass
            
            try:
                self.frame_queue.put_nowait(frame)
            except:
                self.dropped_frames += 1
    
    def publish_frame(self):
        """Publish frame from queue"""
        if self.frame_queue.empty():
            return
        
        try:
            frame = self.frame_queue.get_nowait()
        except:
            return
        
        # Create message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.format = 'jpeg'
        
        # Encode to JPEG (pure Python - no cv_bridge)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        success, encoded = cv2.imencode('.jpg', frame, encode_param)
        
        if success:
            msg.data = encoded.tobytes()
            self.image_pub.publish(msg)
            self.frame_count += 1
    
    def print_stats(self):
        """Stats output"""
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        drop_pct = (self.dropped_frames / (self.frame_count + self.dropped_frames) * 100) if (self.frame_count + self.dropped_frames) > 0 else 0
        
        self.get_logger().info(
            f"Frames: {self.frame_count} | Dropped: {self.dropped_frames} ({drop_pct:.1f}%) | "
            f"FPS: {fps:.1f} | Queue: {self.frame_queue.qsize()}/{QUEUE_SIZE}"
        )
    
    def cleanup(self):
        self.is_capturing = False
        if self.capture_thread:
            self.capture_thread.join(timeout=2.0)
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        pub = CameraPublisher()
        rclpy.spin(pub)
    except KeyboardInterrupt:
        pass
    finally:
        if 'pub' in locals():
            pub.cleanup()
            pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()