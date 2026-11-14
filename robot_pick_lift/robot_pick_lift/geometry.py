"""Rigid transforms using ROS quaternion order (x, y, z, w)."""
import math


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def normalized(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('Quaternion must contain four finite values')
    norm = math.sqrt(sum(v*v for v in q))
    if norm < 1e-12:
        raise ValueError('Quaternion must be nonzero')
    return [v/norm for v in q]


def tool_pose_for_grasp(position, orientation, offset, mounting_rotation):
    """Solve world_T_tool = world_T_grasp * inverse(tool_T_grasp)."""
    for xyz in (position, offset):
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            raise ValueError('Translation must contain three finite values')
    mounting = normalized(mounting_rotation)
    tool_q = multiply(normalized(orientation),
                      [-mounting[0], -mounting[1], -mounting[2], mounting[3]])
    rotated = multiply(multiply(tool_q, list(offset) + [0.0]),
                       [-tool_q[0], -tool_q[1], -tool_q[2], tool_q[3]])[:3]
    return [p-o for p, o in zip(position, rotated)], tool_q


def quaternion_from_rpy(rpy):
    """Fixed-axis roll/pitch/yaw to ROS quaternion."""
    r, p, y = (v / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]
