"""Verify that the FR3 port retains the task and uses consistent model names."""
from pathlib import Path
from types import SimpleNamespace
import math
import runpy
import xml.etree.ElementTree as ET

import pytest
import xacro
from fr3_pick_lift.moveit_pick_lift import MoveItPickLift, downward_pose

ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / 'fr3_2f85_description'


def test_model_and_semantics_match():
    model = ET.fromstring(xacro.process_file(
        str(DESCRIPTION / 'urdf/fr3_gazebo.urdf.xacro'),
        mappings={'robot_type': 'fr3', 'hand': 'false', 'gazebo': 'true',
                  'gazebo_effort': 'false'}).toxml())
    semantic = ET.fromstring(xacro.process_file(
        str(DESCRIPTION / 'config/fr3_2f85.srdf.xacro')).toxml())
    links = {e.get('name') for e in model.findall('link')}
    joints = {e.get('name') for e in model.findall('joint')}
    assert {'world', 'fr3_link0', 'fr3_link8', 'robotiq_85_base_link'} <= links
    assert not any('finger_joint' in j and 'robotiq' not in j for j in joints)
    group = semantic.find("group[@name='fr3_arm']/chain")
    assert group.get('tip_link') == 'fr3_link8'
    for collision in semantic.findall('disable_collisions'):
        assert collision.get('link1') in links
        assert collision.get('link2') in links
    for system in model.findall('ros2_control'):
        assert system.findtext('hardware/plugin') == 'ign_ros2_control/IgnitionSystem'
    assert model.find("joint[@name='robotiq_85_base_joint']/parent").get('link') == 'fr3_link8'


def test_same_world_as_working_ur3e():
    assert (DESCRIPTION / 'worlds/mvp3a_table_cube.sdf').read_bytes() == (
        ROOT / 'ur3e_2f85_description/worlds/mvp3a_table_cube.sdf').read_bytes()


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2])
def test_fingers_point_down(yaw):
    q = downward_pose(0.45, 0.0, 0.185, yaw).orientation
    assert q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w == pytest.approx(1.0)
    assert 1 - 2*(q.x*q.x + q.y*q.y) == pytest.approx(-1.0)


def test_pose_request_uses_fr3_group_and_flange():
    goals = []
    fake = SimpleNamespace(move=object(), action=lambda client, goal: goals.append(goal))
    MoveItPickLift.move_pose(fake, downward_pose(0.45, 0.0, 0.285, 0.0))
    request = goals[0].request
    assert request.group_name == 'fr3_arm'
    assert request.goal_constraints[0].position_constraints[0].link_name == 'fr3_link8'
    assert request.goal_constraints[0].orientation_constraints[0].link_name == 'fr3_link8'


def test_moveit_launch_has_seven_fr3_controller_joints():
    launch = runpy.run_path(str(DESCRIPTION / 'launch/moveit.launch.py'))
    captured = []
    launch['generate_launch_description'].__globals__['Node'] = (
        lambda **kwargs: captured.append(kwargs) or SimpleNamespace())
    launch['generate_launch_description'].__globals__['LaunchDescription'] = lambda entities: entities
    launch['generate_launch_description']()
    params = captured[0]['parameters'][0]
    controller = params['moveit_simple_controller_manager']['fr3_arm_controller']
    assert controller['joints'] == [f'fr3_joint{i}' for i in range(1, 8)]
    assert 'fr3_arm' in params['robot_description_kinematics']
    assert 'fr3_arm' in params['ompl']
    assert 'panda_arm' not in params['ompl']


def test_task_keeps_pick_lift_and_release_sequence(monkeypatch):
    from fr3_pick_lift import moveit_pick_lift as task
    from fr3_pick_lift.gripper import GRIPPER_OPEN, GRIPPER_CLOSE

    poses, commands, holds = [], [], []
    values = dict(block_x=0.45, block_y=0.0, block_z=0.04, tcp_offset_z=0.145,
                  grasp_yaw=0.0, lift_distance=0.10, hold_seconds=3.0)
    fake = SimpleNamespace(
        add_table=lambda: None,
        allow_grasp_contact=lambda: None,
        value=values.__getitem__,
        move_pose=lambda pose, linear=False: poses.append((pose.position.z, linear)),
        move_gripper=lambda position, **kwargs: commands.append(position) or True,
        get_logger=lambda: SimpleNamespace(info=lambda message: None))
    monkeypatch.setattr(task.time, 'sleep', holds.append)
    MoveItPickLift.run(fake)
    assert [z for z, linear in poses] == pytest.approx([0.285, 0.185, 0.285, 0.191, 0.285])
    assert [linear for z, linear in poses] == [False, True, True, True, True]
    assert commands == [GRIPPER_OPEN, GRIPPER_CLOSE, GRIPPER_OPEN]
    assert holds == [3.0]


def test_scene_protects_floor_block_and_preserves_collision_matrix():
    from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
    from moveit_msgs.srv import GetPlanningScene
    matrix = AllowedCollisionMatrix(
        entry_names=['fr3_link1', 'fr3_link2'],
        entry_values=[AllowedCollisionEntry(enabled=[False, True]),
                      AllowedCollisionEntry(enabled=[True, False])])
    applied = []
    service = SimpleNamespace(wait_for_service=lambda **kwargs: True,
                              call_async=lambda request: request)

    def wait(request):
        if isinstance(request, GetPlanningScene.Request):
            return SimpleNamespace(scene=SimpleNamespace(allowed_collision_matrix=matrix))
        applied.append(request)
        return SimpleNamespace(success=True)

    fake = SimpleNamespace(scene=service, get_scene=service, wait=wait,
                           value={'block_x': 0.45, 'block_y': 0.0, 'block_z': 0.04}.__getitem__)
    MoveItPickLift.add_table(fake)
    objects = {obj.id: obj for obj in applied[0].scene.world.collision_objects}
    assert set(objects) == {'pick_table', 'pick_block', 'pick_floor'}
    assert list(objects['pick_floor'].planes[0].coef) == [0.0, 0.0, 1.0, 0.0]
    assert objects['pick_block'].primitive_poses[0].position.z == 0.04
    assert matrix.entry_values[0].enabled[1]
    floor = matrix.entry_names.index('pick_floor')
    base = matrix.entry_names.index('fr3_link0')
    assert matrix.entry_values[floor].enabled[base]
    assert not matrix.entry_values[floor].enabled[0]
