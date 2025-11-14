#!/usr/bin/env python3

"""
Arjuna QR Tracking
Detects QR code with "Arjuna" text, displays zones, publishes tracking commands

Company: NEWRRO TECH LLP
Website: www.newrro.in
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
from pyzbar import pyzbar
import threading
from queue import Queue
import time

# ==================== PARAMETERS ====================
WINDOW_NAME = "Arjuna QR Tracking"
TARGET_QR_TEXT = "Arjuna"  # Case sensitive
ZONE_LEFT_PERCENT = 40     # Left zone: 0-40%
ZONE_CENTER_PERCENT = 20   # Center zone: 40-60%
ZONE_RIGHT_PERCENT = 40    # Right zone: 60-100%

# Colors (BGR format)
COLOR_GREEN = (0, 255, 0)
COLOR_RED = (0, 0, 255)
COLOR_BLUE = (255, 0, 0)
COLOR_YELLOW = (0, 255, 255)
COLOR_WHITE = (255, 255, 255)

class QRTracker(Node):
    def __init__(self):
        super().__init__('arjuna_qr_tracker')
        
        # State
        self.current_frame = None
        self.qr_detected = False
        self.qr_zone = None  # 'left', 'center', 'right'
        self.qr_data = None
        self.qr_bbox = None
        
        # Statistics
        self.frame_count = 0
        self.qr_count = 0
        self.start_time = time.time()
        
        # Frame queue for display
        self.frame_queue = Queue(maxsize=2)
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/arjuna/camera/image_raw/compressed',
            self.image_callback,
            10)
        
        # Publish QR tracking command
        self.qr_cmd_pub = self.create_publisher(
            String,
            '/arjuna/qr_tracking/command',
            10)
        
        # Display thread
        self.is_displaying = True
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        # Stats timer
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("ARJUNA QR TRACKING - ACTIVE")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Target QR: '{TARGET_QR_TEXT}' (case sensitive)")
        self.get_logger().info(f"Zones: LEFT {ZONE_LEFT_PERCENT}% | CENTER {ZONE_CENTER_PERCENT}% | RIGHT {ZONE_RIGHT_PERCENT}%")
        self.get_logger().info("Subscribes: /arjuna/camera/image_raw/compressed")
        self.get_logger().info("Publishes: /arjuna/qr_tracking/command")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Window: Green box = centered | Red box = need alignment")
        self.get_logger().info("Press Q or ESC to quit")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
    
    def image_callback(self, msg):
        """Process incoming camera frames"""
        try:
            # Decode JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                return
            
            self.frame_count += 1
            
            # Process frame for QR detection
            self.process_frame(frame)
            
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")
    
    def process_frame(self, frame):
        """Detect QR codes and determine zone"""
        height, width = frame.shape[:2]
        
        # Calculate zone boundaries
        left_boundary = int(width * ZONE_LEFT_PERCENT / 100)
        right_boundary = int(width * (ZONE_LEFT_PERCENT + ZONE_CENTER_PERCENT) / 100)
        
        # Draw zone partition lines
        cv2.line(frame, (left_boundary, 0), (left_boundary, height), COLOR_YELLOW, 2)
        cv2.line(frame, (right_boundary, 0), (right_boundary, height), COLOR_YELLOW, 2)
        
        # Add zone labels
        cv2.putText(frame, "LEFT", (int(left_boundary/2) - 30, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_YELLOW, 2)
        cv2.putText(frame, "CENTER", (left_boundary + int((right_boundary - left_boundary)/2) - 40, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_YELLOW, 2)
        cv2.putText(frame, "RIGHT", (right_boundary + int((width - right_boundary)/2) - 35, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_YELLOW, 2)
        
        # Detect QR codes
        qr_codes = pyzbar.decode(frame)
        
        # Reset detection state
        self.qr_detected = False
        self.qr_zone = None
        command = "stop"
        
        # Process detected QR codes
        for qr in qr_codes:
            qr_data = qr.data.decode('utf-8')
            
            # Only process QR with target text (case sensitive)
            if qr_data == TARGET_QR_TEXT:
                self.qr_detected = True
                self.qr_data = qr_data
                self.qr_count += 1
                
                # Get QR bounding box
                points = qr.polygon
                if len(points) == 4:
                    # Calculate center of QR code
                    qr_center_x = int(sum([p.x for p in points]) / 4)
                    
                    # Determine which zone QR is in
                    if qr_center_x < left_boundary:
                        self.qr_zone = "left"
                        box_color = COLOR_RED
                        command = "left"
                    elif qr_center_x < right_boundary:
                        self.qr_zone = "center"
                        box_color = COLOR_GREEN
                        command = "center"
                    else:
                        self.qr_zone = "right"
                        box_color = COLOR_RED
                        command = "right"
                    
                    # Draw bounding box
                    pts = np.array([[p.x, p.y] for p in points], np.int32)
                    pts = pts.reshape((-1, 1, 2))
                    cv2.polylines(frame, [pts], True, box_color, 3)
                    
                    # Draw center point
                    cv2.circle(frame, (qr_center_x, int(sum([p.y for p in points]) / 4)), 
                             5, box_color, -1)
                    
                    # Display QR data and zone
                    text_y = int(sum([p.y for p in points]) / 4) - 20
                    cv2.putText(frame, f"QR: {qr_data}", 
                              (qr_center_x - 50, text_y),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                    cv2.putText(frame, f"Zone: {self.qr_zone.upper()}", 
                              (qr_center_x - 50, text_y + 25),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                    
                    # Only process first matching QR
                    break
        
        # Publish tracking command
        cmd_msg = String()
        cmd_msg.data = command
        self.qr_cmd_pub.publish(cmd_msg)
        
        # Add status info at bottom
        status_text = f"Status: {'QR DETECTED' if self.qr_detected else 'NO QR'} | Command: {command.upper()}"
        status_color = COLOR_GREEN if self.qr_detected else COLOR_RED
        cv2.putText(frame, status_text, (10, height - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # Add FPS
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, height - 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_WHITE, 2)
        
        # Add to display queue
        if self.frame_queue.full():
            try:
                self.frame_queue.get_nowait()
            except:
                pass
        
        try:
            self.frame_queue.put_nowait(frame)
        except:
            pass
    
    def display_loop(self):
        """Display processed frames"""
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
                self.get_logger().info("Quit key pressed")
                self.is_displaying = False
                break
        
        cv2.destroyAllWindows()
    
    def print_stats(self):
        """Print statistics"""
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        
        self.get_logger().info(
            f"Stats | Frames: {self.frame_count} | QR Detections: {self.qr_count} | "
            f"FPS: {fps:.1f} | Current: {'QR=' + self.qr_zone.upper() if self.qr_detected else 'NO QR'}"
        )
    
    def cleanup(self):
        """Cleanup resources"""
        self.is_displaying = False
        if self.display_thread:
            self.display_thread.join(timeout=2.0)
        cv2.destroyAllWindows()
        self.get_logger().info("QR Tracker cleaned up")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        tracker = QRTracker()
        
        while tracker.is_displaying and rclpy.ok():
            rclpy.spin_once(tracker, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'tracker' in locals():
            tracker.cleanup()
            tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()