#!/usr/bin/env python3
"""
Arjuna Web Control Server
Provides web interface for robot control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from flask import Flask, render_template_string, jsonify, request
from flask_cors import CORS
import threading
import json

class WebServer(Node):
    def __init__(self):
        super().__init__('web_server')
        
        # Robot state
        self.current_velocity = {'linear': 0.0, 'angular': 0.0}
        self.laser_data = {}
        self.robot_status = "ready"
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        # Flask app
        self.app = Flask(__name__)
        CORS(self.app)
        
        # Setup routes
        self.setup_routes()
        
        self.get_logger().info("Web Server Node Started")
        
    def laser_callback(self, msg):
        """Process LIDAR data"""
        self.laser_data = {
            'front_left': min(msg.ranges[0:130]) if len(msg.ranges) > 130 else 10.0,
            'front_right': min(msg.ranges[721:850]) if len(msg.ranges) > 850 else 10.0,
            'left': min(msg.ranges[231:280]) if len(msg.ranges) > 280 else 10.0,
            'right': min(msg.ranges[571:620]) if len(msg.ranges) > 620 else 10.0,
        }
        
    def setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)
        
        @self.app.route('/api/move', methods=['POST'])
        def move():
            data = request.json
            twist = Twist()
            twist.linear.x = float(data.get('linear', 0.0))
            twist.angular.z = float(data.get('angular', 0.0))
            self.cmd_pub.publish(twist)
            self.current_velocity = {'linear': twist.linear.x, 'angular': twist.angular.z}
            return jsonify({'status': 'ok'})
        
        @self.app.route('/api/status', methods=['GET'])
        def status():
            return jsonify({
                'velocity': self.current_velocity,
                'laser': self.laser_data,
                'status': self.robot_status
            })
        
        @self.app.route('/api/stop', methods=['POST'])
        def stop():
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.current_velocity = {'linear': 0.0, 'angular': 0.0}
            return jsonify({'status': 'stopped'})
    
    def run_server(self):
        """Run Flask server"""
        self.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

# HTML Template
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Arjuna Control Panel</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
            background: rgba(255,255,255,0.1);
            padding: 30px;
            border-radius: 15px;
            backdrop-filter: blur(10px);
        }
        h1 {
            text-align: center;
            margin-bottom: 30px;
        }
        .control-pad {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin: 30px 0;
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
        }
        button:hover {
            background: rgba(255,255,255,0.4);
            transform: scale(1.05);
        }
        button:active {
            transform: scale(0.95);
        }
        .stop-btn {
            background: #e74c3c;
            grid-column: 2;
        }
        .forward-btn { grid-column: 2; }
        .left-btn { grid-column: 1; grid-row: 2; }
        .right-btn { grid-column: 3; grid-row: 2; }
        .backward-btn { grid-column: 2; grid-row: 3; }
        .status-panel {
            margin-top: 30px;
            padding: 20px;
            background: rgba(0,0,0,0.3);
            border-radius: 10px;
        }
        .status-item {
            margin: 10px 0;
            display: flex;
            justify-content: space-between;
        }
        .laser-display {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
            margin-top: 20px;
        }
        .laser-item {
            padding: 10px;
            background: rgba(255,255,255,0.1);
            border-radius: 5px;
            text-align: center;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Arjuna Control Panel</h1>
        
        <div class="control-pad">
            <button class="forward-btn" onclick="move(0.3, 0)">⬆️ Forward</button>
            <button class="left-btn" onclick="move(0, 0.5)">⬅️ Left</button>
            <button class="stop-btn" onclick="stop()">⏹️ STOP</button>
            <button class="right-btn" onclick="move(0, -0.5)">➡️ Right</button>
            <button class="backward-btn" onclick="move(-0.3, 0)">⬇️ Backward</button>
        </div>
        
        <div class="status-panel">
            <h3>Robot Status</h3>
            <div class="status-item">
                <span>Linear Velocity:</span>
                <span id="linear">0.0 m/s</span>
            </div>
            <div class="status-item">
                <span>Angular Velocity:</span>
                <span id="angular">0.0 rad/s</span>
            </div>
            
            <h4>LIDAR Sensors</h4>
            <div class="laser-display">
                <div class="laser-item">
                    <div>Front Left</div>
                    <div id="front_left">--</div>
                </div>
                <div class="laser-item">
                    <div>Front Right</div>
                    <div id="front_right">--</div>
                </div>
                <div class="laser-item">
                    <div>Left</div>
                    <div id="left">--</div>
                </div>
                <div class="laser-item">
                    <div>Right</div>
                    <div id="right">--</div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        function move(linear, angular) {
            fetch('/api/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({linear: linear, angular: angular})
            });
        }
        
        function stop() {
            fetch('/api/stop', {method: 'POST'});
        }
        
        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('linear').textContent = data.velocity.linear.toFixed(2) + ' m/s';
                    document.getElementById('angular').textContent = data.velocity.angular.toFixed(2) + ' rad/s';
                    
                    if (data.laser.front_left) {
                        document.getElementById('front_left').textContent = data.laser.front_left.toFixed(2) + ' m';
                        document.getElementById('front_right').textContent = data.laser.front_right.toFixed(2) + ' m';
                        document.getElementById('left').textContent = data.laser.left.toFixed(2) + ' m';
                        document.getElementById('right').textContent = data.laser.right.toFixed(2) + ' m';
                    }
                });
        }
        
        setInterval(updateStatus, 500);
        
        // Keyboard controls
        document.addEventListener('keydown', (e) => {
            switch(e.key) {
                case 'ArrowUp': case 'w': move(0.3, 0); break;
                case 'ArrowDown': case 's': move(-0.3, 0); break;
                case 'ArrowLeft': case 'a': move(0, 0.5); break;
                case 'ArrowRight': case 'd': move(0, -0.5); break;
                case ' ': stop(); break;
            }
        });
    </script>
</body>
</html>
'''

def main(args=None):
    rclpy.init(args=args)
    node = WebServer()
    
    # Start Flask in separate thread
    server_thread = threading.Thread(target=node.run_server, daemon=True)
    server_thread.start()
    
    node.get_logger().info("Web server running at http://localhost:5000")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()