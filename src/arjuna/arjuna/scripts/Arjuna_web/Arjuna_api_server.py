#!/usr/bin/env python3
"""
Arjuna Simple API Server
RESTful API for robot control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from flask import Flask, jsonify, request
from flask_cors import CORS
import threading

class APIServer(Node):
    def __init__(self):
        super().__init__('api_server')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Flask app
        self.app = Flask(__name__)
        CORS(self.app)
        
        self.setup_api_routes()
        
        self.get_logger().info("API Server Started")
        
    def setup_api_routes(self):
        """Setup API routes"""
        
        @self.app.route('/api/v1/move', methods=['POST'])
        def move():
            """Move robot with given velocities"""
            data = request.json
            twist = Twist()
            twist.linear.x = float(data.get('linear_x', 0.0))
            twist.linear.y = float(data.get('linear_y', 0.0))
            twist.angular.z = float(data.get('angular_z', 0.0))
            self.cmd_pub.publish(twist)
            return jsonify({'status': 'success', 'command': 'move'})
        
        @self.app.route('/api/v1/stop', methods=['POST'])
        def stop():
            """Stop robot"""
            twist = Twist()
            self.cmd_pub.publish(twist)
            return jsonify({'status': 'success', 'command': 'stop'})
        
        @self.app.route('/api/v1/health', methods=['GET'])
        def health():
            """Health check"""
            return jsonify({'status': 'healthy', 'service': 'arjuna_api'})
    
    def run_server(self):
        """Run Flask server"""
        self.app.run(host='0.0.0.0', port=8080, debug=False, use_reloader=False)

def main(args=None):
    rclpy.init(args=args)
    node = APIServer()
    
    server_thread = threading.Thread(target=node.run_server, daemon=True)
    server_thread.start()
    
    node.get_logger().info("API server running at http://localhost:8080")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()