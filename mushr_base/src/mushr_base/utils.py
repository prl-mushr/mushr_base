#!/usr/bin/env python

import numpy as np
import tf
import tf.transformations
from geometry_msgs.msg import Quaternion


def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw


def map_to_world(pose, map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)

    world_pose = np.zeros(pose.shape)

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinate since they will be overwritten

    world_pose[0] = c * pose[0] - s * pose[1]
    world_pose[1] = s * pose[0] + c * pose[1]

    # scale
    world_pose[0] *= float(scale)
    world_pose[1] *= float(scale)

    # translate
    world_pose[0] += map_info.origin.position.x
    world_pose[1] += map_info.origin.position.y
    world_pose[2] += angle

    return world_pose


def world_to_map(pose, map_info):
    # equivalent to map_to_grid(world_to_map(poses))
    # operates in place
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)

    map_pose = np.array(pose)

    # translation
    map_pose[0] -= map_info.origin.position.x
    map_pose[1] -= map_info.origin.position.y

    # scale
    map_pose[0] *= 1.0 / float(scale)
    map_pose[1] *= 1.0 / float(scale)

    # rotation
    c, s = np.cos(angle), np.sin(angle)

    tmp = map_pose[0]
    map_pose[0] = c * map_pose[0] - s * map_pose[1]
    map_pose[1] = s * tmp + c * map_pose[1]
    map_pose[2] += angle

    return map_pose
