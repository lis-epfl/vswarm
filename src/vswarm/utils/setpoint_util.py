#!/usr/bin/env python

import numpy as np
import rospy
from mavros_msgs.msg import PositionTarget

""" Message for SET_POSITION_TARGET_LOCAL_NED
# From: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html

# std_msgs/Header header

# uint8 coordinate_frame
# uint8 FRAME_LOCAL_NED = 1
# uint8 FRAME_LOCAL_OFFSET_NED = 7
# uint8 FRAME_BODY_NED = 8
# uint8 FRAME_BODY_OFFSET_NED = 9

# uint16 type_mask
# uint16 IGNORE_PX = 1  # Position ignore flags
# uint16 IGNORE_PY = 2
# uint16 IGNORE_PZ = 4
# uint16 IGNORE_VX = 8  # Velocity vector ignore flags
# uint16 IGNORE_VY = 16
# uint16 IGNORE_VZ = 32
# uint16 IGNORE_AFX = 64  # Acceleration/Force vector ignore flags
# uint16 IGNORE_AFY = 128
# uint16 IGNORE_AFZ = 256
# uint16 FORCE = 512  # Force in af vector flag (NOT SUPPORTED)
# uint16 IGNORE_YAW = 1024
# uint16 IGNORE_YAW_RATE = 2048

# geometry_msgs/Point position
# geometry_msgs/Vector3 velocity
# geometry_msgs/Vector3 acceleration_or_force
# float32 yaw
# float32 yaw_rate
"""

POSITION_MASK = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048
VELOCITY_MASK = 1 + 2 + 4 + 64 + 128 + 256 + 512 + 2048
ALTITUDE_MASK = 1 + 2 + 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048


def position_to_angle(x, y):
    theta = np.arctan2(y, x)
    if theta < 0:
        return theta + (2 * np.pi)
    return theta


def altitude_target(altitude, yaw=0.0, frame_id='world'):
    pt = PositionTarget()
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = frame_id
    pt.coordinate_frame = 1
    pt.type_mask = POSITION_MASK
    pt.position.z = altitude
    pt.yaw = yaw  # in radians!
    return pt


def position_target(position, yaw=0.0, frame_id='world'):
    pt = PositionTarget()
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = frame_id
    pt.coordinate_frame = 1
    pt.type_mask = POSITION_MASK
    pt.position.x = position[0]
    pt.position.y = position[1]
    pt.position.z = position[2]
    pt.yaw = yaw  # in radians!
    return pt


def velocity_target(velocity, frame_id='world'):
    pt = PositionTarget()
    pt.header.stamp = rospy.Time.now()
    pt.header.frame_id = frame_id
    pt.coordinate_frame = 1
    pt.type_mask = VELOCITY_MASK
    pt.velocity.x = velocity[0]
    pt.velocity.y = velocity[1]
    pt.velocity.z = velocity[2]
    pt.yaw = position_to_angle(velocity[0], velocity[1])
    return pt
