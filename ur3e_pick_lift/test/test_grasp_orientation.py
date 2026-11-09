"""Check the physical grasp direction, including ROS quaternion defaults."""
import math

import pytest

from ur3e_pick_lift.moveit_pick_lift import downward_pose


@pytest.mark.parametrize('yaw', [0.0, 0.5, math.pi, -math.pi / 2])
def test_grasp_points_down(yaw):
    pose = downward_pose(0.45, 0.0, 0.185, yaw)
    q = pose.orientation
    assert q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w == pytest.approx(1.0)
    # Third column of quaternion rotation matrix: local +Z in world frame.
    assert 2*(q.x*q.z + q.w*q.y) == pytest.approx(0.0)
    assert 2*(q.y*q.z - q.w*q.x) == pytest.approx(0.0)
    assert 1 - 2*(q.x*q.x + q.y*q.y) == pytest.approx(-1.0)
    assert pose.position.z == pytest.approx(0.185)
