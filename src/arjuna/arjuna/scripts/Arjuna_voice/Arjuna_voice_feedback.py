#!/usr/bin/env python3
"""
Arjuna Voice Feedback System
Provides audio feedback for voice commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import pyttsx3
import threading

class VoiceFeedback(Node):
    def __init__(self):
        super().__init__('voice_feedback')
        
        # Initialize text-to-speech engine
        self.tts_engine = pyttsx3.init()
        
        # Configure TTS
        self.tts_engine.setProperty('rate', 150)  # Speed of speech
        self.tts_engine.setProperty('volume', 0.9)  # Volume (0.0 to 1.0)
        
        # Try to set voice (female voice if available)
        voices = self.tts_engine.getProperty('voices')
        if len(voices) > 1:
            self.tts_engine.setProperty('voice', voices[1].id)  # Usually female
        
        # Subscribers
        self.recognized_text_sub = self.create_subscription(
            String,
            '/recognized_text',
            self.recognized_text_callback,
            10
        )
        
        self.listening_status_sub = self.create_subscription(
            Bool,
            '/listening_status',
            self.listening_status_callback,
            10
        )
        
        # Feedback messages
        self.feedback_map = {
            'forward': "Moving forward",
            'backward': "Moving backward",
            'left': "Turning left",
            'right': "Turning right",
            'stop': "Stopping",
            'halt': "Halting",
        }
        
        self.get_logger().info("Voice Feedback System Started")
        
        # Welcome message
        self.speak_async("Voice feedback system ready")
        
    def speak_async(self, text):
        """Speak text in separate thread to avoid blocking"""
        def speak_thread():
            try:
                self.tts_engine.say(text)
                self.tts_engine.runAndWait()
            except Exception as e:
                self.get_logger().error(f"TTS error: {e}")
        
        thread = threading.Thread(target=speak_thread, daemon=True)
        thread.start()
        
    def recognized_text_callback(self, msg):
        """Callback for recognized text"""
        text = msg.data.lower()
        self.get_logger().info(f"Recognized: {text}")
        
        # Find matching feedback
        for key, feedback in self.feedback_map.items():
            if key in text:
                self.speak_async(feedback)
                return
        
        # No specific feedback, just acknowledge
        self.speak_async("Command received")
        
    def listening_status_callback(self, msg):
        """Callback for listening status"""
        if msg.data:
            self.get_logger().debug("Robot is listening...")
        else:
            self.get_logger().debug("Robot stopped listening")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceFeedback()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()