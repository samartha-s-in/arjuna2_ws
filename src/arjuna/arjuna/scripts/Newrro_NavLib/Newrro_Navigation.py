#!/usr/bin/env python3
"""
Company Name : NEWRRO TECH
Date         : 2024
ROS Version  : ROS 2
Website      : www.newrro.in
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

# Navigation parameters
YAW_PRECISION = math.pi / 20  # +/- ~9 degrees precision
DIST_PRECISION = 0.05        # Goal point radius in meters (5cm)
LINEAR_VELOCITY = 0.5        # Linear velocity (m/s)
ANGULAR_VELOCITY = 0.8       # Angular velocity (rad/s)

# Global publisher (will be set by navigation nodes)
cmd_pub = None

def set_cmd_publisher(publisher):
    """Set the cmd_vel publisher for navigation functions"""
    global cmd_pub
    cmd_pub = publisher

def normalize_angle(angle):
    """Normalize angle to be between -pi and pi"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def fix_yaw(des_pos, yaw_, position_, state_):
    """Align robot toward goal position"""
    global YAW_PRECISION, cmd_pub
    
    # Calculate desired yaw angle to goal
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    # Create twist message
    twist_msg = Twist()
    
    # Calculate appropriate angular velocity
    if abs(err_yaw) > YAW_PRECISION:
        # Proportional control for smoother rotation
        k_p = 1.0
        angular_z = k_p * err_yaw
        
        # Clamp to max velocity
        angular_z = max(min(angular_z, ANGULAR_VELOCITY), -ANGULAR_VELOCITY)
        twist_msg.angular.z = angular_z
        
        print(f"Aligning: Current yaw={yaw_:.2f}, Desired={desired_yaw:.2f}, Error={err_yaw:.2f}")
    else:
        print("Yaw aligned with goal")
        state_ = 1
        print(f"Robot State: {state_}")
    
    if cmd_pub:
        cmd_pub.publish(twist_msg)
    
    return yaw_, position_, state_

def go_straight_ahead(des_pos, yaw_, position_, state_):
    """Move straight toward goal while maintaining orientation"""
    global YAW_PRECISION, DIST_PRECISION, cmd_pub
    
    # Calculate errors
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    err_pos = math.sqrt((des_pos.y - position_.y)**2 + (des_pos.x - position_.x)**2)
    
    # Check if goal reached
    if err_pos <= DIST_PRECISION:
        print("Goal reached!")
        state_ = 2
        print(f"Robot State: {state_}")
        return yaw_, position_, state_
    
    # If orientation error too large, realign
    if abs(err_yaw) > YAW_PRECISION:
        print("Orientation error too large, realigning")
        state_ = 0
        print(f"Robot State: {state_}")
        return yaw_, position_, state_
    
    # Move forward with minor corrections
    twist_msg = Twist()
    twist_msg.linear.x = LINEAR_VELOCITY
    
    # Minor orientation correction
    if err_yaw != 0:
        k_p = 0.5
        twist_msg.angular.z = k_p * err_yaw
    
    if cmd_pub:
        cmd_pub.publish(twist_msg)
    
    return yaw_, position_, state_

def stop():
    """Stop the robot"""
    global cmd_pub
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    if cmd_pub:
        cmd_pub.publish(twist_msg)

def move_straight():
    """Move robot straight ahead"""
    global LINEAR_VELOCITY, cmd_pub
    twist_msg = Twist()
    twist_msg.linear.x = LINEAR_VELOCITY
    if cmd_pub:
        cmd_pub.publish(twist_msg)

def turn_left():
    """Turn robot left"""
    global ANGULAR_VELOCITY, cmd_pub
    twist_msg = Twist()
    twist_msg.angular.z = ANGULAR_VELOCITY
    if cmd_pub:
        cmd_pub.publish(twist_msg)

def turn_right():
    """Turn robot right"""
    global ANGULAR_VELOCITY, cmd_pub
    twist_msg = Twist()
    twist_msg.angular.z = -ANGULAR_VELOCITY
    if cmd_pub:
        cmd_pub.publish(twist_msg)