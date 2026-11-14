"""Exercise the shared task with transport replaced by recording doubles."""
from pathlib import Path
from types import SimpleNamespace
import runpy

from launch import LaunchContext
from moveit_msgs.msg import AllowedCollisionEntry
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import pytest
import yaml

from robot_pick_lift.moveit_pick_lift import MoveItPickLift

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('robot', ['fr3', 'ur3e'])
def test_task_order_and_targets(robot):
    params = yaml.safe_load((ROOT / f'{robot}_2f85_description/config/pick_lift.yaml')
                            .read_text())['robot_pick_lift']['ros__parameters']
    params.update(block_x=0.45, block_y=0.0, block_z=0.04, grasp_yaw=0.0,
                  lift_distance=0.10, hold_seconds=0.0)
    events = []
    fake = SimpleNamespace(
        value=params.__getitem__, add_table=lambda: events.append('scene'),
        allow_grasp_contact=lambda: events.append('contact'),
        check_targets=lambda targets: events.append(('preflight', [phase for phase, _ in targets])),
        move_pose=lambda pose, linear=False: events.append(('move', pose.position.z, linear)),
        move_gripper=lambda position, **kw: events.append(('gripper', position)) or True,
        get_logger=lambda: SimpleNamespace(info=lambda message: None))
    MoveItPickLift.run(fake)
    assert events == [
        'scene', ('preflight', ['PREGRASP', 'GRASP', 'LIFT', 'RELEASE']), ('move', 0.28500000000000003, False), ('gripper', params['gripper_open']),
        'contact', ('move', 0.185, True), ('gripper', params['gripper_close']),
        ('move', 0.28500000000000003, True),
        ('move', 0.185 + params['release_clearance'], True),
        ('gripper', params['gripper_open']), ('move', 0.28500000000000003, True)]


def test_scene_preserves_existing_collision_exclusions():
    query_result = GetPlanningScene.Response()
    matrix = query_result.scene.allowed_collision_matrix
    matrix.entry_names = ['existing_a', 'existing_b']
    matrix.entry_values = [AllowedCollisionEntry(enabled=[False, True]),
                           AllowedCollisionEntry(enabled=[True, False])]
    recorded = []
    scene = SimpleNamespace(wait_for_service=lambda **kw: True,
                            call_async=lambda req: recorded.append(req) or ApplyPlanningScene.Response(success=True))
    query = SimpleNamespace(wait_for_service=lambda **kw: True,
                            call_async=lambda req: query_result)
    params = dict(reference_frame='world', block_x=0.45, block_y=0., block_z=0.04,
                  grasp_yaw=0., table_size=[0.6, 0.8, 0.04],
                  table_position=[0.45, 0., 0.], table_quaternion=[0., 0., 0., 1.],
                  block_size=[0.04, 0.04, 0.04], block_quaternion=[0., 0., 0., 1.], floor_z=0.,
                  floor_contact_links=['base_link', 'base_link_inertia'])
    fake = SimpleNamespace(scene=scene, get_scene=query, value=params.__getitem__, wait=lambda x: x)
    MoveItPickLift.add_table(fake)
    result = recorded[0].scene
    assert {obj.id for obj in result.world.collision_objects} == {'pick_table', 'pick_block', 'pick_floor'}
    matrix = result.allowed_collision_matrix
    assert matrix.entry_values[0].enabled[1]
    floor = matrix.entry_names.index('pick_floor')
    for link in params['floor_contact_links']:
        assert matrix.entry_values[floor].enabled[matrix.entry_names.index(link)]
    assert not matrix.entry_values[floor].enabled[0]
    MoveItPickLift.allow_grasp_contact(fake)
    assert recorded[-1].scene.world.collision_objects[0].id == 'pick_block'
    assert recorded[-1].scene.world.collision_objects[0].operation == b'\x01'  # ROS Humble exposes byte-valued REMOVE as bytes


@pytest.mark.parametrize('robot', ['fr3', 'ur3e'])
def test_launch_loads_selected_description(robot):
    module = runpy.run_path(str(ROOT / 'robot_pick_lift/launch/pick_lift.launch.py'))
    context = LaunchContext()
    for entity in module['generate_launch_description']().entities[:-1]:
        entity.execute(context)
    context.launch_configurations['robot'] = robot
    context.launch_configurations['arm_config'] = str(ROOT / f'{robot}_2f85_description/config/arm.yaml')
    context.launch_configurations['gripper_config'] = str(ROOT / f'{robot}_2f85_description/config/gripper.yaml')
    context.launch_configurations['environment_config'] = str(ROOT / 'robot_pick_lift/config/environment.yaml')
    actions = module['launch_task'](context)
    assert len(actions) == 1
    context.launch_configurations['start_moveit'] = 'true'
    assert len(module['launch_task'](context)) == 3


def test_launch_supports_custom_package_without_robot_registry(tmp_path, monkeypatch):
    module = runpy.run_path(str(ROOT / 'robot_pick_lift/launch/pick_lift.launch.py'))
    (tmp_path / 'config').mkdir()
    for name in ('arm.yaml', 'gripper.yaml'):
        (tmp_path / 'config' / name).write_text((ROOT / 'fr3_2f85_description/config' / name).read_text())
    seen = []
    from robot_pick_lift import configuration
    original_share = configuration.share
    def resolve(package):
        if package == 'new_arm_description':
            seen.append(package)
            return str(tmp_path)
        return original_share(package)
    monkeypatch.setattr(configuration, 'share', resolve)
    module['launch_task'].__globals__['get_package_share_directory'] = lambda package: str(tmp_path)
    context = LaunchContext()
    for entity in module['generate_launch_description']().entities[:-1]:
        entity.execute(context)
    context.launch_configurations['description_package'] = 'new_arm_description'
    context.launch_configurations['environment_config'] = str(ROOT / 'robot_pick_lift/config/environment.yaml')
    assert len(module['launch_task'](context)) == 1
    assert seen == ['new_arm_description']
