"""World/configuration consistency, real Xacro assembly, and failure handling."""
from pathlib import Path
import runpy
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPlanningScene, GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes
import pytest

from robot_pick_lift.bringup import render_assembly, moveit_parameters
from robot_pick_lift.configuration import load_configuration, environment_parameters, read_yaml
from robot_pick_lift.moveit_pick_lift import MoveItPickLift
from robot_pick_lift.world import world_sdf

ROOT = Path(__file__).resolve().parents[2]
ENVIRONMENT = ROOT / 'robot_pick_lift/config/environment.yaml'


def configuration(robot):
    directory = ROOT / f'{robot}_2f85_description/config'
    return load_configuration(arm_config=str(directory / 'arm.yaml'),
                              gripper_config=str(directory / 'gripper.yaml'),
                              environment_config=str(ENVIRONMENT))


@pytest.mark.parametrize('robot', ['fr3', 'ur3e'])
def test_assembly_and_moveit_match(robot, tmp_path):
    arm, gripper, _, parameters = configuration(robot)
    arm['pose']['position'] = [0.1, 0.2, 0.3]
    arm['pose']['orientation_rpy'] = [0.0, 0.0, 0.5]
    gripper['mount']['position'] = [0.01, 0.02, 0.03]
    description, semantic, controllers = render_assembly(arm, gripper, tmp_path)
    xml = ET.fromstring(description)
    mounts = [joint for joint in xml.findall('joint')
              if joint.find('parent').get('link') == 'world']
    assert any(j.find('origin').get('xyz') == '0.1 0.2 0.3' for j in mounts)
    gripper_joint = xml.find("joint[@name='robotiq_85_base_joint']")
    assert gripper_joint.find('origin').get('xyz') == '0.01 0.02 0.03'
    assert str(tmp_path / 'controllers.yaml') in description
    params = moveit_parameters(arm, description, semantic, controllers)
    assert params['robot_description'] == description
    assert parameters['planning_group'] in params['robot_description_kinematics']
    assert arm['controller'] in params['moveit_simple_controller_manager']['controller_names']
    assert set(controllers['controller_manager']['ros__parameters']) == {
        'update_rate', 'use_sim_time', arm['controller'], arm['joint_state_controller'], gripper['controller']}


def test_world_geometry_matches_task_parameters():
    env = read_yaml(ENVIRONMENT)
    env['table'].update(size=[0.7, 0.9, 0.1], position=[0.5, 0.1, 0.2], orientation_rpy=[0.1, 0.2, 0.3])
    env['block'].update(size=[0.03, 0.05, 0.07], position=[0.6, 0.1, 0.3], orientation_rpy=[0., 0., 0.7])
    params = environment_parameters(env)
    world = ET.fromstring(world_sdf(env))
    for key, name in [('table', 'table'), ('block', 'green_block')]:
        model = world.find(f"world/model[@name='{name}']")
        size = [float(v) for v in model.findtext('link/collision/geometry/box/size').split()]
        pose = [float(v) for v in model.findtext('pose').split()]
        assert size == params[f'{key}_size']
        assert pose[:3] == env[key]['position']
        assert pose[3:] == env[key]['orientation_rpy']
    inertia = world.find("world/model[@name='green_block']/link/inertial/inertia")
    assert float(inertia.findtext('ixx')) == pytest.approx(0.05*(0.05**2+0.07**2)/12)
    assert params['grasp_yaw'] == 0.7


@pytest.mark.parametrize('field,value', [('size', [-1, 1, 1]), ('position', [0, float('nan'), 0])])
def test_invalid_environment_fails_before_launch(field, value):
    env = read_yaml(ENVIRONMENT)
    env['table'][field] = value
    with pytest.raises(ValueError):
        world_sdf(env)


def test_failed_spawn_does_not_activate_controllers():
    module = runpy.run_path(str(ROOT / 'robot_pick_lift/launch/simulation.launch.py'))
    marker = object()
    callback = module['after_success']([marker], 'spawn')
    assert callback(SimpleNamespace(returncode=0), None) == [marker]
    assert marker not in callback(SimpleNamespace(returncode=1), None)


@pytest.mark.parametrize('reachable', [True, False])
def test_ik_preflight_restores_scene_and_stops_on_failure(reachable):
    response = GetPlanningScene.Response()
    response.scene.robot_state.joint_state.name = ['gripper_joint']
    response.scene.robot_state.joint_state.position = [0.75]
    result = GetPositionIK.Response()
    result.error_code.val = MoveItErrorCodes.SUCCESS if reachable else MoveItErrorCodes.NO_IK_SOLUTION
    result.solution = response.scene.robot_state
    calls = []
    from moveit_msgs.msg import CollisionObject
    fake = SimpleNamespace(
        value={'gripper_joint': 'gripper_joint', 'gripper_open': 0., 'planning_group': 'arm',
               'tool_link': 'tool', 'reference_frame': 'world', 'ik_timeout': 2.}.__getitem__,
        wait=lambda future, *args: future,
        get_scene=SimpleNamespace(call_async=lambda request: response),
        ik=SimpleNamespace(wait_for_service=lambda **kw: True,
                           call_async=lambda request: calls.append(request) or result),
        allow_grasp_contact=lambda: calls.append('remove'),
        block_proxy=CollisionObject(id='pick_block'),
        scene=SimpleNamespace(call_async=lambda request: calls.append('restore') or SimpleNamespace(success=True)))
    if reachable:
        MoveItPickLift.check_targets(fake, [('GRASP', Pose())])
        assert calls[1].ik_request.avoid_collisions
        assert calls[1].ik_request.robot_state.joint_state.position[0] == 0.
    else:
        with pytest.raises(RuntimeError, match='GRASP: no collision-free IK'):
            MoveItPickLift.check_targets(fake, [('GRASP', Pose())])
    assert calls[0] == 'remove'
    assert calls[-1] == 'restore'


def test_unreachable_target_sends_no_motion_or_gripper_command():
    _, _, _, params = configuration('fr3')
    calls = []
    def reject(targets):
        raise RuntimeError('unreachable')
    fake = SimpleNamespace(value=params.__getitem__, add_table=lambda: None,
                           check_targets=reject, move_pose=lambda *a, **kw: calls.append('arm'),
                           move_gripper=lambda *a, **kw: calls.append('gripper'))
    with pytest.raises(RuntimeError, match='unreachable'):
        MoveItPickLift.run(fake)
    assert calls == []


def test_mount_configuration_composes_grasp_transform(tmp_path):
    import math
    import yaml
    arm, gripper, _, _ = configuration('fr3')
    gripper['mount']['position'] = [0.01, 0.02, 0.03]
    gripper['mount']['orientation_rpy'] = [0., math.pi/2, 0.]
    for name, data in [('arm', arm), ('gripper', gripper)]:
        (tmp_path / f'{name}.yaml').write_text(yaml.safe_dump(data))
    _, _, _, parameters = load_configuration(arm_config=str(tmp_path / 'arm.yaml'),
                                            environment_config=str(ENVIRONMENT))
    assert parameters['tool_to_grasp_xyz'] == pytest.approx([0.155, 0.02, 0.03])
    assert parameters['tool_to_grasp_quaternion'] == pytest.approx([0., math.sin(math.pi/4), 0., math.cos(math.pi/4)])
