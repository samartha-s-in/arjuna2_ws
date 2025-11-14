#!/usr/bin/env python3

import numpy
from transforms3d import quaternions

def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    """Return quaternion from Euler angles and axis sequence.
    """
    return quaternions.euler2quat(ai, aj, ak, axes)

def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.
    """
    return quaternions.quat2euler(quaternion, axes)
    