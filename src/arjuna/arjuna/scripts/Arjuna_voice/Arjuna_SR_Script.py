cat > Arjuna_SR_Script.py << 'EOF'
#!/usr/bin/env python3
"""
Arjuna Speech Recognition Script
Core speech recognition functionality for voice commands
Company: NEWRRO TECH
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import speech_recognition as sr
import time

class ArjunaSpeechRecognition(Node):
    def __init__(self):
        super().__init__('arjuna_speech_recognition')
        
        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Recognition settings
        self.recognizer.energy_threshold = 4000
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.8
        
        # Robot control parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.is_listening = True
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.recognized_text_pub = self.create_publisher(String, '/recognized_text', 10)
        self.listening_status_pub = self.create_publisher(Bool, '/listening_status', 10)
        
        # Command dictionary
        self.command_map = {
            # Movement commands
            'forward': {'linear': self.linear_speed, 'angular': 0.0},
            'go forward': {'linear': self.linear_speed, 'angular': 0.0},
            'move forward': {'linear': self.linear_speed, 'angular': 0.0},
            'ahead': {'linear': self.linear_speed, 'angular': 0.0},
            
            'backward': {'linear': -self.linear_speed, 'angular': 0.0},
            'go backward': {'linear': -self.linear_speed, 'angular': 0.0},
            'move backward': {'linear': -self.linear_speed, 'angular': 0.0},
            'back': {'linear': -self.linear_speed, 'angular': 0.0},
            'reverse': {'linear': -self.linear_speed, 'angular': 0.0},
            
            'left': {'linear': 0.0, 'angular': self.angular_speed},
            'turn left': {'linear': 0.0, 'angular': self.angular_speed},
            'go left': {'linear': 0.0, 'angular': self.angular_speed},
            'rotate left': {'linear': 0.0, 'angular': self.angular_speed},
            
            'right': {'linear': 0.0, 'angular': -self.angular_speed},
            'turn right': {'linear': 0.0, 'angular': -self.angular_speed},
            'go right': {'linear': 0.0, 'angular': -self.angular_speed},
            'rotate right': {'linear': 0.0, 'angular': -self.angular_speed},
            
            'stop': {'linear': 0.0, 'angular': 0.0},
            'halt': {'linear': 0.0, 'angular': 0.0},
            'freeze': {'linear': 0.0, 'angular': 0.0},
            'wait': {'linear': 0.0, 'angular': 0.0},
        }
        
        # Calibrate microphone
        self.calibrate_microphone()
        
        self.get_logger().info("===========================================")
        self.get_logger().info("Arjuna Speech Recognition Started")
        self.get_logger().info("===========================================")
        self.get_logger().info("Available commands:")
        self.get_logger().info("  - forward/backward/left/right")
        self.get_logger().info("  - stop/halt")
        self.get_logger().info("  - exit/quit (to shutdown)")
        self.get_logger().info("===========================================")
        
    def calibrate_microphone(self):
        """Calibrate microphone for ambient noise"""
        self.get_logger().info("Calibrating microphone for ambient noise...")
        self.get_logger().info("Please remain quiet for 2 seconds...")
        
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
            self.get_logger().info("Calibration complete!")
            self.get_logger().info(f"Energy threshold: {self.recognizer.energy_threshold}")
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {e}")
            
    def publish_listening_status(self, is_listening):
        """Publish listening status"""
        msg = Bool()
        msg.data = is_listening
        self.listening_status_pub.publish(msg)
        
    def publish_recognized_text(self, text):
        """Publish recognized text"""
        msg = String()
        msg.data = text
        self.recognized_text_pub.publish(msg)
        
    def publish_velocity(self, linear, angular):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        
    def process_command(self, command_text):
        """Process recognized command"""
        command_text = command_text.lower().strip()
        
        self.get_logger().info(f"Processing: '{command_text}'")
        
        # Check for exit commands
        if any(word in command_text for word in ['exit', 'quit', 'shutdown']):
            self.get_logger().info("Exit command received")
            self.publish_velocity(0.0, 0.0)
            self.is_listening = False
            rclpy.shutdown()
            return True
        
        # Check for exact command match
        if command_text in self.command_map:
            cmd = self.command_map[command_text]
            self.publish_velocity(cmd['linear'], cmd['angular'])
            self.get_logger().info(f"Executing: {command_text}")
            return True
        
        # Check for partial matches
        for key, cmd in self.command_map.items():
            if key in command_text:
                self.publish_velocity(cmd['linear'], cmd['angular'])
                self.get_logger().info(f"Executing: {key}")
                return True
        
        # Command not recognized
        self.get_logger().warn(f"Command not recognized: '{command_text}'")
        return False
        
    def listen_for_command(self):
        """Listen for a single voice command"""
        try:
            with self.microphone as source:
                self.get_logger().info("Listening for command...")
                self.publish_listening_status(True)
                
                # Listen with timeout
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=4)
                
                self.publish_listening_status(False)
                self.get_logger().info("Processing audio...")
                
                try:
                    # Recognize speech using Google Speech Recognition
                    text = self.recognizer.recognize_google(audio)
                    self.get_logger().info(f"Recognized: '{text}'")
                    
                    # Publish recognized text
                    self.publish_recognized_text(text)
                    
                    # Process command
                    self.process_command(text)
                    
                    return True
                    
                except sr.UnknownValueError:
                    self.get_logger().warn("Could not understand audio")
                    return False
                    
                except sr.RequestError as e:
                    self.get_logger().error(f"Speech recognition service error: {e}")
                    return False
                    
        except sr.WaitTimeoutError:
            self.get_logger().debug("Listening timeout - no speech detected")
            self.publish_listening_status(False)
            return False
            
        except Exception as e:
            self.get_logger().error(f"Error during listening: {e}")
            self.publish_listening_status(False)
            return False
            
    def run(self):
        """Main run loop"""
        self.get_logger().info("Ready for voice commands!")
        
        while self.is_listening and rclpy.ok():
            self.listen_for_command()
            time.sleep(0.5)  # Brief pause between listening cycles

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArjunaSpeechRecognition()
        node.run()
    except KeyboardInterrupt:
        print("\nSpeech recognition interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Stop robot before shutdown
        try:
            node.publish_velocity(0.0, 0.0)
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()