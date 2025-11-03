#!/usr/bin/env python3
"""
Simple Voice Command System
Basic voice commands without continuous listening
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import speech_recognition as sr

class SimpleVoiceCommand(Node):
    def __init__(self):
        super().__init__('simple_voice_command')
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Simple Voice Command Started")
        self.get_logger().info("Say: forward, backward, left, right, stop")
        
    def listen_and_execute(self):
        """Listen for one command and execute"""
        with sr.Microphone() as source:
            print("\nListening for command...")
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            
            try:
                audio = self.recognizer.listen(source, timeout=3)
                command = self.recognizer.recognize_google(audio).lower()
                print(f"Command: {command}")
                
                twist = Twist()
                
                if 'forward' in command:
                    twist.linear.x = 0.3
                    print("Moving forward")
                elif 'backward' in command or 'back' in command:
                    twist.linear.x = -0.3
                    print("Moving backward")
                elif 'left' in command:
                    twist.angular.z = 0.5
                    print("Turning left")
                elif 'right' in command:
                    twist.angular.z = -0.5
                    print("Turning right")
                elif 'stop' in command:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    print("Stopping")
                else:
                    print("Unknown command")
                    return
                
                self.cmd_pub.publish(twist)
                
            except sr.UnknownValueError:
                print("Could not understand audio")
            except sr.RequestError as e:
                print(f"Error: {e}")
            except sr.WaitTimeoutError:
                print("Listening timeout")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleVoiceCommand()
    
    try:
        while rclpy.ok():
            node.listen_and_execute()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()