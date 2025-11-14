#!/usr/bin/env python3

"""
Arjuna Camera Subscriber - NO cv_bridge
Uses pure Python for maximum compatibility
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import threading
from queue import Queue
import time

WINDOW_NAME = "Arjuna Camera"
QUEUE_SIZE = 5
SHOW_FPS = True

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('arjuna_camera_subscriber')
        
        # Stats
        self.frame_count = 0
        self.dropped_frames = 0
        self.start_time = time.time()
        self.last_frame_time = time.time()
        self.current_fps = 0.0
        
        # Frame queue
        self.frame_queue = Queue(maxsize=QUEUE_SIZE)
        
        # Subscribe
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/arjuna/camera/image_raw/compressed',
            self.image_callback,
            10)
        
        # Display thread
        self.is_displaying = True
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        # Stats timer
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("ARJUNA CAMERA SUBSCRIBER - ACTIVE")
        self.get_logger().info("Subscribed: /arjuna/camera/image_raw/compressed")
        self.get_logger().info("Window: 'Arjuna Camera'")
        self.get_logger().info("Press Q or ESC to quit")
        self.get_logger().info("=" * 60)
    
    def image_callback(self, msg):
        """Receive compressed image"""
        try:
            # Decode JPEG (pure Python - no cv_bridge)
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                return
            
            # Calculate FPS
            current_time = time.time()
            time_diff = current_time - self.last_frame_time
            if time_diff > 0:
                self.current_fps = 1.0 / time_diff
            self.last_frame_time = current_time
            
            # Add FPS overlay
            if SHOW_FPS:
                cv2.putText(frame, f"FPS: {self.current_fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Drop old if full
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                    self.dropped_frames += 1
                except:
                    pass
            
            try:
                self.frame_queue.put_nowait(frame)
                self.frame_count += 1
            except:
                self.dropped_frames += 1
                
        except Exception as e:
            self.get_logger().error(f"Decode error: {e}")
    
    def display_loop(self):
        """Display thread"""
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        
        while self.is_displaying:
            if not self.frame_queue.empty():
                try:
                    frame = self.frame_queue.get_nowait()
                    cv2.imshow(WINDOW_NAME, frame)
                except:
                    pass
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q') or key == 27:
                self.is_displaying = False
                break
        
        cv2.destroyAllWindows()
    
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
        self.is_displaying = False
        if self.display_thread:
            self.display_thread.join(timeout=2.0)
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        sub = CameraSubscriber()
        
        while sub.is_displaying and rclpy.ok():
            rclpy.spin_once(sub, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        if 'sub' in locals():
            sub.cleanup()
            sub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()