#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Instructions for the user
msg = """
Control Your Robot! (Stateful Mode)
---------------------------
The robot will continue moving with the last command until you issue a new one.
Press SPACE to stop the robot.

Current Mode: Differential

--- Differential Controls ---
    ↑     ^     →    
      <   >
    ↓     v     ←

    ^ / v: Forward / Backward
    ← / →: Turn Left / Turn Right
    ↑ / →: Forward-Turn Left / Right
    ↓ / ←: Backward-Turn Left / Right
    
--- Mecanum Controls ---
    w     s     a
    ↑     ↓     d
    q     z     e

    w/s: Forward / Backward
    a/d: Strafe Left / Right
    q/e: Diagonal Forward Left / Right
    z/c: Diagonal Backward Left / Right
    x:   Turn in place (use j/l to set direction)

--- Global Controls ---
- SPACE: Emergency Stop
- t: Toggle between Differential and Mecanum modes.
- i/k: Increase/Decrease linear speed by 10%
- j/l: Increase/Decrease angular speed by 10%
- CTRL-C to quit
"""

# Key bindings for speed adjustments
speedBindings = {
    'i': (1.1, 1.0),
    'k': (0.9, 1.0),
    'j': (1.0, 1.1),
    'l': (1.0, 0.9),
}

# Key bindings for differential drive mode
differentialBindings = {
    'up':    (1.0, 0.0, 0.0, 0.0),
    'down':  (-1.0, 0.0, 0.0, 0.0),
    'left':  (0.0, 0.0, 0.0, 1.0),
    'right': (0.0, 0.0, 0.0, -1.0),
    'u':     (1.0, 0.0, 0.0, 1.0),
    'o':     (1.0, 0.0, 0.0, -1.0),
    'm':     (-1.0, 0.0, 0.0, 1.0),
    '.':     (-1.0, 0.0, 0.0, -1.0),
}

# Key bindings for mecanum drive mode
mecanumBindings = {
    'w': (1.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0),
    'a': (0.0, -1.0, 0.0, 0.0),
    'd': (0.0, 1.0, 0.0, 0.0),
    'q': (1.0, -1.0, 0.0, 0.0),
    'e': (1.0, 1.0, 0.0, 0.0),
    'z': (-1.0, -1.0, 0.0, 0.0),
    'c': (-1.0, 1.0, 0.0, 0.0),
    'x': (0.0, 0.0, 0.0, 1.0),
}

def getKey():
    """Gets a single key press from the user."""
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            extra = sys.stdin.read(2)
            if extra == '[A':
                key = 'up'
            elif extra == '[B':
                key = 'down'
            elif extra == '[C':
                key = 'right'
            elif extra == '[D':
                key = 'left'
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    """Returns a string displaying the current speed and turn rate."""
    return "currently:\tspeed {}\tturn {}".format(round(speed, 2), round(turn, 2))

class ArjunaController(Node):
    def __init__(self):
        super().__init__('Arjuna_Controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # State variables
        self.speed = self.declare_parameter('speed', 0.5).value
        self.turn = self.declare_parameter('turn', 1.0).value
        
        self.x, self.y, self.z, self.th = 0.0, 0.0, 0.0, 0.0
        self.mode = 'differential'
        self.twist = Twist()

    def publish_twist(self):
        self.twist.linear.x = self.x * self.speed
        self.twist.linear.y = self.y * self.speed
        self.twist.linear.z = self.z * self.speed
        self.twist.angular.z = self.th * self.turn
        self.publisher_.publish(self.twist)

    def run_controller(self):
        print(msg)
        self.get_logger().info(vels(self.speed, self.turn))

        while rclpy.ok():
            key = getKey()

            if key in speedBindings:
                self.speed *= speedBindings[key][0]
                self.turn *= speedBindings[key][1]
                self.get_logger().info(vels(self.speed, self.turn))
            elif key == 't':
                self.x, self.y, self.z, self.th = 0.0, 0.0, 0.0, 0.0
                self.publish_twist()
                if self.mode == 'differential':
                    self.mode = 'mecanum'
                    self.get_logger().info("Mode switched to: Mecanum")
                else:
                    self.mode = 'differential'
                    self.get_logger().info("Mode switched to: Differential")
            elif key == ' ':
                self.x, self.y, self.z, self.th = 0.0, 0.0, 0.0, 0.0
                self.get_logger().info("EMERGENCY STOP")
            elif key == '\x03': # CTRL-C
                self.get_logger().info("Exiting...")
                self.x, self.y, self.z, self.th = 0.0, 0.0, 0.0, 0.0
                self.publish_twist()
                break
            else:
                if self.mode == 'differential' and key in differentialBindings:
                    self.x, self.y, self.z, self.th = differentialBindings[key]
                elif self.mode == 'mecanum' and key in mecanumBindings:
                    self.x, self.y, self.z, self.th = mecanumBindings[key]
                else:
                    pass
            
            self.publish_twist()

def run_arjuna(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    arjuna_node = ArjunaController()
    
    try:
        arjuna_node.run_controller()
    except Exception as e:
        arjuna_node.get_logger().error(f'An error occurred: {e}')
    finally:
        arjuna_node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    run_arjuna()