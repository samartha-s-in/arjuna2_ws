#!/usr/bin/env python3
"""
qr_recog_debug.py
Subscribe to /arjuna/camera/image_raw/compressed and print diagnostics.
Tries pyzbar if available, else uses cv2.QRCodeDetector.
Saves /tmp/qr_debug.jpg for inspection.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import time
TOPIC = '/arjuna/camera/image_raw/compressed'
SHOW_WINDOW = True

# try import pyzbar for fallback multi-QR detection
try:
    from pyzbar import pyzbar
    HAS_PYZBAR = True
except Exception:
    HAS_PYZBAR = False

class QRDebug(Node):
    def __init__(self):
        super().__init__('qr_recog_debug')
        self.sub = self.create_subscription(CompressedImage, TOPIC, self.cb, 10)
        self.detector = cv2.QRCodeDetector()
        self.last_saved = 0
        self.get_logger().info(f'Listening on {TOPIC}  pyzbar={"yes" if HAS_PYZBAR else "no"}')

    def cb(self, msg: CompressedImage):
        now = time.time()
        # decode compressed image
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warn('cv2.imdecode returned None')
                return
        except Exception as e:
            self.get_logger().error(f'Failed to decode image: {e}')
            return

        h,w = img.shape[:2]
        self.get_logger().debug(f'Frame received: shape={img.shape} len_data={len(msg.data)}')

        found_any = False
        # first try pyzbar (if installed)
        if HAS_PYZBAR:
            barcodes = pyzbar.decode(img)
            if barcodes:
                found_any = True
                for barcode in barcodes:
                    txt = barcode.data.decode('utf-8', errors='replace')
                    pts = barcode.polygon
                    self.get_logger().info(f'pyzbar: {txt}')
                    # draw polygon
                    pts_t = [(p.x, p.y) for p in pts]
                    for i in range(len(pts_t)):
                        cv2.line(img, pts_t[i], pts_t[(i+1)%len(pts_t)], (0,255,0), 2)
                    cv2.putText(img, txt, (pts_t[0][0], max(15, pts_t[0][1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2)

        # fallback to OpenCV QRCodeDetector (single QR)
        if not found_any:
            try:
                text, pts, straight_qrcode = self.detector.detectAndDecode(img)
            except Exception as e:
                self.get_logger().warn(f'detectAndDecode exception: {e}')
                text, pts = None, None
            if text:
                found_any = True
                self.get_logger().info(f'OpenCV QR: {text}')
                if pts is not None and len(pts) > 0:
                    try:
                        pts = pts.astype(int).reshape((-1,2))
                        for i in range(len(pts)):
                            p1 = tuple(pts[i]); p2 = tuple(pts[(i+1)%len(pts)])
                            cv2.line(img, p1, p2, (255,0,0), 2)
                        cv2.putText(img, text, (pts[0][0], max(15, pts[0][1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0),2)
                    except Exception as e:
                        self.get_logger().warn(f'Failed to draw pts: {e}')

        # save debug image occasionally
        if now - self.last_saved > 1.0:
            try:
                cv2.imwrite('/tmp/qr_debug.jpg', img)
                self.get_logger().info(f'Saved /tmp/qr_debug.jpg  shape={img.shape}  found_any={found_any}')
                self.last_saved = now
            except Exception as e:
                self.get_logger().warn(f'Failed to save debug jpg: {e}')

        if SHOW_WINDOW:
            # overlay text
            overlay = img.copy()
            cv2.putText(overlay, f'{w}x{h}  pyzbar={HAS_PYZBAR}  found={found_any}', (10,20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),1)
            cv2.imshow('QR Debug', overlay)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = QRDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
