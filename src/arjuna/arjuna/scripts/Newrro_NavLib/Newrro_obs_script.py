#!/usr/bin/env python3
"""
Company Name : NEWRRO TECH
ROS Version  : ROS 2
Obstacle avoidance functions with persistent avoidance
"""

from geometry_msgs.msg import Twist
import time

# Navigation parameters
LINEAR_VELOCITY = 0.5
ANGULAR_VELOCITY = 0.8
OBSTACLE_DIST_THRESHOLD = 0.3
SAFE_DISTANCE = 0.5  # Distance considered safe after avoidance

# Track avoidance state
avoidance_state = {
    'avoiding': False,
    'avoidance_type': None,
    'start_time': 0,
    'min_avoidance_time': 2.0  # Minimum time to continue avoidance maneuver
}

def is_path_clear(regions):
    """Check if path is clear of obstacles"""
    if not regions:
        return True
    
    # Path is clear only if all critical regions are safe
    return (regions['front_L'] > SAFE_DISTANCE and 
            regions['front_R'] > SAFE_DISTANCE and
            regions['fleft'] > SAFE_DISTANCE and
            regions['fright'] > SAFE_DISTANCE)

def is_obstacle_detected(regions):
    """Check if any obstacle is within threshold"""
    if not regions:
        return False
    
    return (regions['front_L'] < OBSTACLE_DIST_THRESHOLD or 
            regions['front_R'] < OBSTACLE_DIST_THRESHOLD or
            regions['fleft'] < OBSTACLE_DIST_THRESHOLD or
            regions['fright'] < OBSTACLE_DIST_THRESHOLD or
            regions['left'] < OBSTACLE_DIST_THRESHOLD or
            regions['right'] < OBSTACLE_DIST_THRESHOLD)

def avoid_obstacle(regions):
    """
    Generate avoidance command with persistent behavior.
    Continues avoidance maneuver until path is completely clear.
    """
    global avoidance_state
    
    twist_msg = Twist()
    current_time = time.time()
    
    # Check obstacle configurations
    front_obstacle = (regions['front_L'] < OBSTACLE_DIST_THRESHOLD or 
                     regions['front_R'] < OBSTACLE_DIST_THRESHOLD)
    left_obstacle = (regions['fleft'] < OBSTACLE_DIST_THRESHOLD or 
                    regions['left'] < OBSTACLE_DIST_THRESHOLD)
    right_obstacle = (regions['fright'] < OBSTACLE_DIST_THRESHOLD or 
                     regions['right'] < OBSTACLE_DIST_THRESHOLD)
    
    # If we're already avoiding, check if we should continue
    if avoidance_state['avoiding']:
        elapsed = current_time - avoidance_state['start_time']
        
        # Continue current avoidance maneuver if:
        # 1. Minimum avoidance time hasn't elapsed, OR
        # 2. Path is still not clear
        if elapsed < avoidance_state['min_avoidance_time'] or not is_path_clear(regions):
            # Continue previous maneuver
            maneuver = avoidance_state['avoidance_type']
            
            if maneuver == 'back_up':
                twist_msg.linear.x = -0.1
                twist_msg.angular.z = 0.0
                print("Continuing: Backing up from obstacles")
                
            elif maneuver == 'turn_right':
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = -ANGULAR_VELOCITY
                print("Continuing: Turning right")
                
            elif maneuver == 'turn_left':
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = ANGULAR_VELOCITY
                print("Continuing: Turning left")
                
            elif maneuver == 'adjust_right':
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = -ANGULAR_VELOCITY * 0.5
                print("Continuing: Adjusting right")
                
            elif maneuver == 'adjust_left':
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = ANGULAR_VELOCITY * 0.5
                print("Continuing: Adjusting left")
                
            return twist_msg
        else:
            # Avoidance complete
            print("Obstacle avoidance complete - path clear")
            avoidance_state['avoiding'] = False
            avoidance_state['avoidance_type'] = None
            return None
    
    # Start new avoidance maneuver based on obstacle configuration
    if front_obstacle and left_obstacle and right_obstacle:
        # Surrounded - back up
        print("NEW AVOIDANCE: Surrounded by obstacles - backing up")
        twist_msg.linear.x = -0.1
        twist_msg.angular.z = 0.0
        avoidance_state['avoiding'] = True
        avoidance_state['avoidance_type'] = 'back_up'
        avoidance_state['start_time'] = current_time
        avoidance_state['min_avoidance_time'] = 3.0  # Back up longer
        
    elif front_obstacle and left_obstacle:
        # Front and left blocked - turn right
        print("NEW AVOIDANCE: Front and left blocked - turning right")
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = -ANGULAR_VELOCITY
        avoidance_state['avoiding'] = True
        avoidance_state['avoidance_type'] = 'turn_right'
        avoidance_state['start_time'] = current_time
        avoidance_state['min_avoidance_time'] = 2.0
        
    elif front_obstacle and right_obstacle:
        # Front and right blocked - turn left
        print("NEW AVOIDANCE: Front and right blocked - turning left")
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = ANGULAR_VELOCITY
        avoidance_state['avoiding'] = True
        avoidance_state['avoidance_type'] = 'turn_left'
        avoidance_state['start_time'] = current_time
        avoidance_state['min_avoidance_time'] = 2.0
        
    elif front_obstacle:
        # Only front blocked - choose direction with more space
        if regions['left'] > regions['right']:
            print("NEW AVOIDANCE: Front blocked - turning left (more space)")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = ANGULAR_VELOCITY
            avoidance_state['avoidance_type'] = 'turn_left'
        else:
            print("NEW AVOIDANCE: Front blocked - turning right (more space)")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -ANGULAR_VELOCITY
            avoidance_state['avoidance_type'] = 'turn_right'
        
        avoidance_state['avoiding'] = True
        avoidance_state['start_time'] = current_time
        avoidance_state['min_avoidance_time'] = 2.0
        
    elif left_obstacle:
        # Left side blocked - adjust right while moving
        print("NEW AVOIDANCE: Left obstacle - adjusting right")
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = -ANGULAR_VELOCITY * 0.5
        avoidance_state['avoiding'] = True
        avoidance_state['avoidance_type'] = 'adjust_right'
        avoidance_state['start_time'] = current_time
        avoidance_state['min_avoidance_time'] = 1.5
        
    elif right_obstacle:
        # Right side blocked - adjust left while moving
        print("NEW AVOIDANCE: Right obstacle - adjusting left")
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = ANGULAR_VELOCITY * 0.5
        avoidance_state['avoiding'] = True
        avoidance_state['avoidance_type'] = 'adjust_left'
        avoidance_state['start_time'] = current_time
        avoidance_state['min_avoidance_time'] = 1.5
        
    else:
        # No obstacles - shouldn't reach here
        return None
    
    return twist_msg

def reset_avoidance_state():
    """Reset avoidance state - call when starting new navigation task"""
    global avoidance_state
    avoidance_state['avoiding'] = False
    avoidance_state['avoidance_type'] = None
    avoidance_state['start_time'] = 0