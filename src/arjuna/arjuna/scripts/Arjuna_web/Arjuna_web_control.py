#!/usr/bin/env python3
"""
Arjuna Advanced Web Control Interface
Comprehensive web interface with video streaming, sensor data, and control
Company: NEWRRO TECH
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Float32
from flask import Flask, render_template_string, jsonify, request, Response
from flask_cors import CORS
import threading
import cv2
import numpy as np
import json
from datetime import datetime

class ArjunaWebControl(Node):
    def __init__(self):
        super().__init__('arjuna_web_control')
        
        # Robot state
        self.current_velocity = {'linear': 0.0, 'angular': 0.0}
        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.laser_data = {
            'front_left': 10.0,
            'front_right': 10.0,
            'left': 10.0,
            'right': 10.0,
            'fleft': 10.0,
            'fright': 10.0
        }
        self.camera_frame = None
        self.depth_value = 0.0
        self.qr_data = ""
        self.cpu_usage = 0.0
        self.ram_usage = 0.0
        
        # Control parameters
        self.default_linear_speed = 0.3
        self.default_angular_speed = 0.5
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        self.odom_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot_pose_ekf/odom_combined',
            self.odom_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/frames', self.camera_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Float32, '/oak/depth_value', self.depth_callback, 10
        )
        self.qr_sub = self.create_subscription(
            String, '/qr_data', self.qr_callback, 10
        )
        self.cpu_sub = self.create_subscription(
            Float32, '/CPU', self.cpu_callback, 10
        )
        self.ram_sub = self.create_subscription(
            Float32, '/RAM', self.ram_callback, 10
        )
        
        # Flask app
        self.app = Flask(__name__)
        CORS(self.app)
        
        # Setup routes
        self.setup_routes()
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Arjuna Web Control Started")
        self.get_logger().info("===========================================")
        
    def laser_callback(self, msg):
        """Process LIDAR data"""
        try:
            self.laser_data = {
                'front_left': float(min(min(msg.ranges[0:130]), 10.0)),
                'front_right': float(min(min(msg.ranges[721:850]), 10.0)),
                'fleft': float(min(min(msg.ranges[131:230]), 10.0)),
                'fright': float(min(min(msg.ranges[621:720]), 10.0)),
                'left': float(min(min(msg.ranges[231:280]), 10.0)),
                'right': float(min(min(msg.ranges[571:620]), 10.0)),
            }
        except Exception as e:
            self.get_logger().error(f"LIDAR callback error: {e}")
            
    def odom_callback(self, msg):
        """Process odometry data"""
        try:
            self.position['x'] = float(msg.pose.pose.position.x)
            self.position['y'] = float(msg.pose.pose.position.y)
            
            # Calculate yaw from quaternion
            orientation = msg.pose.pose.orientation
            siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
            cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
            self.position['theta'] = float(np.arctan2(siny_cosp, cosy_cosp))
        except Exception as e:
            self.get_logger().error(f"Odom callback error: {e}")
            
    def camera_callback(self, msg):
        """Process camera frame"""
        try:
            # Convert ROS Image to numpy array
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )
            self.camera_frame = frame.copy()
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")
            
    def depth_callback(self, msg):
        """Process depth data"""
        self.depth_value = float(msg.data)
        
    def qr_callback(self, msg):
        """Process QR data"""
        self.qr_data = msg.data
        
    def cpu_callback(self, msg):
        """Process CPU usage"""
        self.cpu_usage = float(msg.data)
        
    def ram_callback(self, msg):
        """Process RAM usage"""
        self.ram_usage = float(msg.data)
        
    def generate_camera_feed(self):
        """Generate camera feed for streaming"""
        while True:
            if self.camera_frame is not None:
                try:
                    # Encode frame as JPEG
                    ret, buffer = cv2.imencode('.jpg', self.camera_frame)
                    frame = buffer.tobytes()
                    
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                except Exception as e:
                    self.get_logger().error(f"Frame encoding error: {e}")
            else:
                # Send blank frame
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                ret, buffer = cv2.imencode('.jpg', blank)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                       
    def setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            return render_template_string(ADVANCED_HTML_TEMPLATE)
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(
                self.generate_camera_feed(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        @self.app.route('/api/move', methods=['POST'])
        def move():
            try:
                data = request.json
                twist = Twist()
                twist.linear.x = float(data.get('linear', 0.0))
                twist.angular.z = float(data.get('angular', 0.0))
                self.cmd_pub.publish(twist)
                self.current_velocity = {
                    'linear': twist.linear.x,
                    'angular': twist.angular.z
                }
                return jsonify({'status': 'ok'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400
        
        @self.app.route('/api/status', methods=['GET'])
        def status():
            return jsonify({
                'velocity': self.current_velocity,
                'position': self.position,
                'laser': self.laser_data,
                'depth': self.depth_value,
                'qr': self.qr_data,
                'cpu': self.cpu_usage,
                'ram': self.ram_usage,
                'timestamp': datetime.now().isoformat()
            })
        
        @self.app.route('/api/stop', methods=['POST'])
        def stop():
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.current_velocity = {'linear': 0.0, 'angular': 0.0}
            return jsonify({'status': 'stopped'})
        
        @self.app.route('/api/set_speed', methods=['POST'])
        def set_speed():
            try:
                data = request.json
                self.default_linear_speed = float(data.get('linear', 0.3))
                self.default_angular_speed = float(data.get('angular', 0.5))
                return jsonify({'status': 'ok'})
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400
    
    def run_server(self):
        """Run Flask server"""
        self.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

# Advanced HTML Template with video streaming
ADVANCED_HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Arjuna Advanced Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
        }
        .header h1 {
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .container {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            max-width: 1400px;
            margin: 0 auto;
        }
        .panel {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            backdrop-filter: blur(10px);
            box-shadow: 0 8px 32px rgba(0,0,0,0.3);
        }
        .panel h2 {
            margin-bottom: 15px;
            border-bottom: 2px solid rgba(255,255,255,0.3);
            padding-bottom: 10px;
        }
        
        /* Video Feed */
        .video-container {
            grid-column: 1 / -1;
            position: relative;
        }
        .video-feed {
            width: 100%;
            border-radius: 10px;
            box-shadow: 0 4px 16px rgba(0,0,0,0.3);
        }
        .video-overlay {
            position: absolute;
            top: 20px;
            left: 20px;
            background: rgba(0,0,0,0.7);
            padding: 10px;
            border-radius: 5px;
        }
        
        /* Control Pad */
        .control-pad {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin: 20px 0;
        }
        button {
            padding: 20px;
            font-size: 18px;
            border: none;
            border-radius: 10px;
            background: rgba(255,255,255,0.2);
            color: white;
            cursor: pointer;
            transition: all 0.3s;
            font-weight: bold;
        }
        button:hover {
            background: rgba(255,255,255,0.4);
            transform: scale(1.05);
        }
        button:active {
            transform: scale(0.95);
        }
        .stop-btn {
            background: #e74c3c !important;
            grid-column: 2;
        }
        .forward-btn { grid-column: 2; grid-row: 1; }
        .left-btn { grid-column: 1; grid-row: 2; }
        .right-btn { grid-column: 3; grid-row: 2; }
        .backward-btn { grid-column: 2; grid-row: 3; }
        
        /* Sensor Grid */
        .sensor-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin-top: 15px;
        }
        .sensor-item {
            padding: 15px;
            background: rgba(0,0,0,0.3);
            border-radius: 8px;
            text-align: center;
        }
        .sensor-value {
            font-size: 1.5em;
            font-weight: bold;
            margin-top: 5px;
        }
        .sensor-label {
            font-size: 0.9em;
            opacity: 0.8;
        }
        
        /* Position Display */
        .position-display {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin-top: 15px;
        }
        
        /* Status Indicators */
        .status-indicator {
            display: flex;
            justify-content: space-between;
            padding: 10px;
            margin: 5px 0;
            background: rgba(0,0,0,0.3);
            border-radius: 5px;
        }
        
        /* Speed Controls */
        .speed-control {
            margin: 15px 0;
        }
        .speed-slider {
            width: 100%;
            margin: 10px 0;
        }
        
        @media (max-width: 768px) {
            .container {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🤖 ARJUNA Advanced Control Panel</h1>
        <p>Real-time Robot Monitoring & Control</p>
    </div>
    
    <div class="container">
        <!-- Video Feed -->
        <div class="panel video-container">
            <h2>📹 Live Camera Feed</h2>
            <div style="position: relative;">
                <img src="/video_feed" class="video-feed" alt="Camera Feed">
                <div class="video-overlay">
                    <div>QR: <span id="qr-status">--</span></div>
                    <div>Depth: <span id="depth-value">--</span> cm</div>
                </div>
            </div>
        </div>
        
        <!-- Control Panel -->
        <div class="panel">
            <h2>🎮 Robot Control</h2>
            <div class="control-pad">
                <button class="forward-btn" onmousedown="move(0.3, 0)" onmouseup="stop()">
                    ⬆️<br>Forward
                </button>
                <button class="left-btn" onmousedown="move(0, 0.5)" onmouseup="stop()">
                    ⬅️<br>Left
                </button>
                <button class="stop-btn" onclick="stop()">
                    ⏹️<br>STOP
                </button>
                <button class="right-btn" onmousedown="move(0, -0.5)" onmouseup="stop()">
                    ➡️<br>Right
                </button>
                <button class="backward-btn" onmousedown="move(-0.3, 0)" onmouseup="stop()">
                    ⬇️<br>Backward
                </button>
            </div>
            
            <div class="speed-control">
                <label>Linear Speed: <span id="linear-speed">0.3</span> m/s</label>
                <input type="range" class="speed-slider" min="0.1" max="0.8" step="0.1" value="0.3" 
                       oninput="updateSpeed(this.value, 'linear')">
                
                <label>Angular Speed: <span id="angular-speed">0.5</span> rad/s</label>
                <input type="range" class="speed-slider" min="0.2" max="1.0" step="0.1" value="0.5"
                       oninput="updateSpeed(this.value, 'angular')">
            </div>
        </div>
        
        <!-- LIDAR Sensors -->
        <div class="panel">
            <h2>📡 LIDAR Sensors</h2>
            <div class="sensor-grid">
                <div class="sensor-item">
                    <div class="sensor-label">Front Left</div>
                    <div class="sensor-value" id="front_left">--</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-label">Front Right</div>
                    <div class="sensor-value" id="front_right">--</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-label">Left</div>
                    <div class="sensor-value" id="left">--</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-label">Right</div>
                    <div class="sensor-value" id="right">--</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-label">F-Left</div>
                    <div class="sensor-value" id="fleft">--</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-label">F-Right</div>
                    <div class="sensor-value" id="fright">--</div>
                </div>
            </div>
        </div>
        
        <!-- Robot Status -->
        <div class="panel">
            <h2>📊 Robot Status</h2>
            
            <h3 style="margin-top: 15px;">Position</h3>
            <div class="position-display">
                <div class="sensor-item">
                    <div class="sensor-label">X</div>
                    <div class="sensor-value" id="pos-x">0.0</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-label">Y</div>
                    <div class="sensor-value" id="pos-y">0.0</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-label">θ</div>
                    <div class="sensor-value" id="pos-theta">0.0</div>
                </div>
            </div>
            
            <h3 style="margin-top: 15px;">Velocity</h3>
            <div class="status-indicator">
                <span>Linear:</span>
                <span id="linear-vel">0.0 m/s</span>
            </div>
            <div class="status-indicator">
                <span>Angular:</span>
                <span id="angular-vel">0.0 rad/s</span>
            </div>
            
            <h3 style="margin-top: 15px;">System</h3>
            <div class="status-indicator">
                <span>CPU Usage:</span>
                <span id="cpu-usage">0%</span>
            </div>
            <div class="status-indicator">
                <span>RAM Usage:</span>
                <span id="ram-usage">0%</span>
            </div>
        </div>
    </div>
    
    <script>
        let linearSpeed = 0.3;
        let angularSpeed = 0.5;
        
        function move(linear, angular) {
            fetch('/api/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    linear: linear * (linearSpeed / 0.3),
                    angular: angular * (angularSpeed / 0.5)
                })
            });
        }
        
        function stop() {
            fetch('/api/stop', {method: 'POST'});
        }
        
        function updateSpeed(value, type) {
            if (type === 'linear') {
                linearSpeed = parseFloat(value);
                document.getElementById('linear-speed').textContent = value;
            } else {
                angularSpeed = parseFloat(value);
                document.getElementById('angular-speed').textContent = value;
            }
        }
        
        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    // Velocity
                    document.getElementById('linear-vel').textContent = 
                        data.velocity.linear.toFixed(2) + ' m/s';
                    document.getElementById('angular-vel').textContent = 
                        data.velocity.angular.toFixed(2) + ' rad/s';
                    
                    // Position
                    document.getElementById('pos-x').textContent = data.position.x.toFixed(2);
                    document.getElementById('pos-y').textContent = data.position.y.toFixed(2);
                    document.getElementById('pos-theta').textContent = data.position.theta.toFixed(2);
                    
                    // LIDAR
                    document.getElementById('front_left').textContent = 
                        data.laser.front_left.toFixed(2) + 'm';
                    document.getElementById('front_right').textContent = 
                        data.laser.front_right.toFixed(2) + 'm';
                    document.getElementById('left').textContent = 
                        data.laser.left.toFixed(2) + 'm';
                    document.getElementById('right').textContent = 
                        data.laser.right.toFixed(2) + 'm';
                    document.getElementById('fleft').textContent = 
                        data.laser.fleft.toFixed(2) + 'm';
                    document.getElementById('fright').textContent = 
                        data.laser.fright.toFixed(2) + 'm';
                    
                    // Camera overlay
                    document.getElementById('qr-status').textContent = 
                        data.qr || 'No QR';
                    document.getElementById('depth-value').textContent = 
                        data.depth.toFixed(1);
                    
                    // System
                    document.getElementById('cpu-usage').textContent = 
                        data.cpu.toFixed(1) + '%';
                    document.getElementById('ram-usage').textContent = 
                        data.ram.toFixed(1) + '%';
                });
        }
        
        // Update status every 500ms
        setInterval(updateStatus, 500);
        
        // Keyboard controls
        document.addEventListener('keydown', (e) => {
            switch(e.key) {
                case 'ArrowUp': case 'w': move(1, 0); break;
                case 'ArrowDown': case 's': move(-1, 0); break;
                case 'ArrowLeft': case 'a': move(0, 1); break;
                case 'ArrowRight': case 'd': move(0, -1); break;
            }
        });
        
        document.addEventListener('keyup', (e) => {
            if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','w','a','s','d'].includes(e.key)) {
                stop();
            }
        });
        
        // Prevent accidental page close
        window.addEventListener('beforeunload', (e) => {
            stop();
        });
    </script>
</body>
</html>
'''

def main(args=None):
    rclpy.init(args=args)
    node = ArjunaWebControl()
    
    # Start Flask in separate thread
    server_thread = threading.Thread(target=node.run_server, daemon=True)
    server_thread.start()
    
    node.get_logger().info("="*50)
    node.get_logger().info("Web Control Interface Running")
    node.get_logger().info("="*50)
    node.get_logger().info("Access at: http://localhost:5000")
    node.get_logger().info("Or from network: http://<robot-ip>:5000")
    node.get_logger().info("="*50)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()