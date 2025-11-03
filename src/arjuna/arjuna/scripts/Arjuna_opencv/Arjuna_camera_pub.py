#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import depthai as dai
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

class CustomCvBridge:
    """Custom cv_bridge for Python 3"""
    def cv2_to_imgmsg(self, cv_image, encoding="bgr8"):
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = encoding
        img_msg.is_bigendian = 0
        
        if encoding == "bgr8":
            img_msg.step = cv_image.shape[1] * 3
            img_msg.data = cv_image.tobytes()
        elif encoding == "mono8":
            img_msg.step = cv_image.shape[1]
            img_msg.data = cv_image.tobytes()
        
        return img_msg

class OAKPublisher(Node):
    def __init__(self):
        super().__init__('oak_publisher')
        
        self.bridge = CustomCvBridge()
        
        # Publishers
        self.frames_pub = self.create_publisher(Image, '/frames', 2)
        self.depth_pub = self.create_publisher(Float32, '/oak/depth_value', 2)
        
        # Setup OAK-D pipeline
        self.pipeline = self.setup_oak_pipeline()
        
        # Connect to device
        self.device = dai.Device(self.pipeline)
        self.get_logger().info("Connected to OAK-D camera")
        
        # Output queues
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=2, blocking=False)
        self.q_depth = self.device.getOutputQueue(name="depth", maxSize=2, blocking=False)
        
        # Variables
        self.min_depth = 100  # 10cm in mm
        self.last_valid_depth = 0.0
        
        # Timer (30Hz)
        self.timer = self.create_timer(1.0/30.0, self.publish_data)
        
    def setup_oak_pipeline(self):
        """Setup OAK-D pipeline"""
        pipeline = dai.Pipeline()
        
        # RGB Camera
        camRgb = pipeline.create(dai.node.ColorCamera)
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setVideoSize(640, 480)
        camRgb.setFps(30)
        
        camRgb.video.link(xoutRgb.input)
        
        # Stereo depth
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        
        xoutDepth.setStreamName("depth")
        
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoLeft.setFps(30)
        
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoRight.setFps(30)
        
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        stereo.depth.link(xoutDepth.input)
        
        return pipeline
        
    def publish_data(self):
        """Publish camera and depth data"""
        # Get RGB frame
        in_rgb = self.q_rgb.tryGet()
        
        if in_rgb is not None:
            rgb_frame = in_rgb.getCvFrame()
            
            try:
                ros_image = self.bridge.cv2_to_imgmsg(rgb_frame, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "oak_camera"
                
                self.frames_pub.publish(ros_image)
                
            except Exception as e:
                self.get_logger().error(f"Error converting image: {e}")
        
        # Get depth data
        in_depth = self.q_depth.tryGet()
        
        if in_depth is not None:
            depth_frame = in_depth.getFrame()
            
            h, w = depth_frame.shape
            center_x, center_y = w // 2, h // 2
            
            center_depth = depth_frame[center_y, center_x]
            
            if center_depth < self.min_depth:
                small_region = depth_frame[center_y-1:center_y+2, center_x-1:center_x+2]
                valid_depths = small_region[small_region >= self.min_depth]
                if valid_depths.size > 0:
                    center_depth = float(np.median(valid_depths))
            
            if center_depth >= self.min_depth:
                center_depth_cm = center_depth / 10.0
                self.last_valid_depth = center_depth_cm
                
                depth_msg = Float32()
                depth_msg.data = center_depth_cm
                self.depth_pub.publish(depth_msg)
            else:
                if self.last_valid_depth > 0:
                    depth_msg = Float32()
                    depth_msg.data = self.last_valid_depth
                    self.depth_pub.publish(depth_msg)
                else:
                    depth_msg = Float32()
                    depth_msg.data = -1.0
                    self.depth_pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OAKPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()