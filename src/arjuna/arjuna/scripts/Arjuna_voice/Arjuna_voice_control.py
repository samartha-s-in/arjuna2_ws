#!/usr/bin/env python3
"""
Arjuna Voice Control System
Enables voice commands for robot control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import speech_recognition as sr
import pyttsx3
import threading

class VoiceControl(Node):
    def __init__(self):
        super().__init__('voice_control')
        
        # Initialize text-to-speech
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)
        self.tts_engine.setProperty('volume', 0.9)
        
        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Adjust for ambient noise
        with self.microphone as source:
            self.get_logger().info("Calibrating for ambient noise... Please wait.")
            self.recognizer.adjust_for_ambient_noise(source, duration=2)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/voice_status', 10)
        
        # Robot state
        self.is_active = True
        self.current_speed = 0.3
        
        # Voice commands mapping
        self.commands = {
            'forward': self.move_forward,
            'go forward': self.move_forward,
            'move forward': self.move_forward,
            'backward': self.move_backward,
            'go backward': self.move_backward,
            'move backward': self.move_backward,
            'back': self.move_backward,
            'left': self.turn_left,
            'turn left': self.turn_left,
            'go left': self.turn_left,
            'right': self.turn_right,
            'turn right': self.turn_right,
            'go right': self.turn_right,
            'stop': self.stop_robot,
            'halt': self.stop_robot,
            'faster': self.increase_speed,
            'speed up': self.increase_speed,
            'slower': self.decrease_speed,
            'slow down': self.decrease_speed,
            'exit': self.exit_voice_control,
            'quit': self.exit_voice_control,
            'shutdown': self.exit_voice_control,
        }
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Voice Control System Started")
        self.get_logger().info("===========================================")
        self.speak("Voice control system activated")
        
        # Start listening in separate thread
        self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listen_thread.start()
        
    def speak(self, text):
        """Text-to-speech output"""
        self.get_logger().info(f"Speaking: {text}")
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")
    
    def publish_status(self, status):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        
    def move_forward(self):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = self.current_speed
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.speak("Moving forward")
        self.publish_status("moving_forward")
        
    def move_backward(self):
        """Move robot backward"""
        twist = Twist()
        twist.linear.x = -self.current_speed
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.speak("Moving backward")
        self.publish_status("moving_backward")
        
    def turn_left(self):
        """Turn robot left"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.cmd_pub.publish(twist)
        self.speak("Turning left")
        self.publish_status("turning_left")
        
    def turn_right(self):
        """Turn robot right"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.5
        self.cmd_pub.publish(twist)
        self.speak("Turning right")
        self.publish_status("turning_right")
        
    def stop_robot(self):
        """Stop robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.speak("Stopping")
        self.publish_status("stopped")
        
    def increase_speed(self):
        """Increase robot speed"""
        self.current_speed = min(self.current_speed + 0.1, 0.8)
        self.speak(f"Speed increased to {self.current_speed:.1f}")
        self.publish_status(f"speed_{self.current_speed:.1f}")
        
    def decrease_speed(self):
        """Decrease robot speed"""
        self.current_speed = max(self.current_speed - 0.1, 0.1)
        self.speak(f"Speed decreased to {self.current_speed:.1f}")
        self.publish_status(f"speed_{self.current_speed:.1f}")
        
    def exit_voice_control(self):
        """Exit voice control"""
        self.stop_robot()
        self.speak("Voice control shutting down")
        self.is_active = False
        self.publish_status("shutdown")
        rclpy.shutdown()
        
    def process_command(self, command):
        """Process voice command"""
        command = command.lower().strip()
        self.get_logger().info(f"Processing command: {command}")
        
        # Check for exact matches
        if command in self.commands:
            self.commands[command]()
            return True
        
        # Check for partial matches
        for key, func in self.commands.items():
            if key in command:
                func()
                return True
        
        # Command not recognized
        self.speak("Command not recognized")
        self.publish_status("unknown_command")
        return False
        
    def listen_loop(self):
        """Main listening loop"""
        self.get_logger().info("Listening for voice commands...")
        self.speak("Ready for voice commands")
        
        while self.is_active and rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info("Listening...")
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=3)
                
                try:
                    # Recognize speech
                    command = self.recognizer.recognize_google(audio)
                    self.get_logger().info(f"Heard: {command}")
                    
                    # Process command
                    self.process_command(command)
                    
                except sr.UnknownValueError:
                    self.get_logger().debug("Could not understand audio")
                except sr.RequestError as e:
                    self.get_logger().error(f"Speech recognition error: {e}")
                    
            except sr.WaitTimeoutError:
                # Timeout - continue listening
                pass
            except Exception as e:
                self.get_logger().error(f"Listening error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VoiceControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nVoice control interrupted")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
