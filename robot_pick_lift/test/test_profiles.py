"""Check robot profiles and grasp transforms without starting controllers."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

from robot_pick_lift.geometry import multiply, tool_pose_for_grasp

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('robot', ['fr3', 'ur3e'])
def test_profile_matches_semantics(robot):
    directory = ROOT / f'{robot}_2f85_description'
    p = yaml.safe_load((directory / 'config/pick_lift.yaml').read_text())[
        'robot_pick_lift']['ros__parameters']
    semantic = ET.fromstring(xacro.process_file(
        str(directory / f'config/{robot}_2f85.srdf.xacro')).toxml())
    group = semantic.find(f"group[@name='{p['planning_group']}']")
    assert group is not None
    assert group.find('chain').get('tip_link') == p['tool_link']
    xyz, _ = tool_pose_for_grasp([0.45, 0, 0.04], [1, 0, 0, 0],
                                p['tool_to_grasp_xyz'], p['tool_to_grasp_quaternion'])
    assert xyz == pytest.approx([0.45, 0, 0.185])


def test_rotated_mount_transform():
    # Nontrivial mounting rotation and lateral displacement must reconstruct
    # the desired grasp pose when composing world_T_tool and tool_T_grasp.
    desired = [0, 0, math.sin(0.3), math.cos(0.3)]
    mount = [math.sin(0.2), 0, 0, math.cos(0.2)]
    offset = [0.02, -0.03, 0.145]
    target = [0.4, 0.1, 0.04]
    xyz, q = tool_pose_for_grasp(target, desired, offset, mount)
    assert multiply(q, mount) == pytest.approx(desired)
    rotated = multiply(multiply(q, offset + [0]), [-q[0], -q[1], -q[2], q[3]])
    assert [a+b for a, b in zip(xyz, rotated)] == pytest.approx(target)


@pytest.mark.parametrize('offset,rotation', [([0, 0], [0, 0, 0, 1]),
                                            ([0, 0, float('nan')], [0, 0, 0, 1]),
                                            ([0, 0, 0], [0, 0, 0, 0])])
def test_invalid_transform_rejected(offset, rotation):
    with pytest.raises(ValueError):
        tool_pose_for_grasp([0, 0, 0], [0, 0, 0, 1], offset, rotation)
