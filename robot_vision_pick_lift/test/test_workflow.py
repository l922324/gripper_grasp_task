"""Check measured-pose handoff and no-motion behavior without live ROS nodes."""
import json
from pathlib import Path
import runpy
from types import SimpleNamespace

import numpy as np
import pytest
from launch import LaunchContext
from robot_pick_lift.moveit_pick_lift import MoveItPickLift
from robot_vision_pick_lift.vision_pick_lift import VisionPickLift

ROOT = Path(__file__).resolve().parents[1]


def logger():
    return SimpleNamespace(info=lambda message: None)


@pytest.mark.parametrize('perception_only,execute', [(True,True),(False,False),(False,True)])
def test_default_workflow_detects_then_executes(perception_only, execute):
    cloud, center, yaw = np.zeros((100,3)), np.array([.49,-.14,.04]), -.11
    calls = []
    params = dict(perception_only=perception_only, execute=execute, block_size=[.04]*3,
                  max_pose_drift=.005, max_yaw_drift_degrees=5.)
    def observe(after=-1.):
        calls.append(('observe',after))
        return cloud,center,yaw,1.,[.04,.04]
    fake = SimpleNamespace(value=params.__getitem__,observe=observe,
                            publish_observation=lambda *args: calls.append('output'),
                            execute_detected_pose=lambda p,y: calls.append(('execute',p.tolist(),y)))
    VisionPickLift.run(fake)
    if perception_only or not execute:
        assert calls == [('observe',-1.),'output']
    else:
        assert calls[-1] == ('execute',center.tolist(),yaw)
        assert calls.index('output') < len(calls)-1


def test_changed_pose_stops_before_motion():
    params = dict(perception_only=False,execute=True,block_size=[.04]*3,max_pose_drift=.005,max_yaw_drift_degrees=5.)
    samples = iter([(np.zeros((100,3)),np.array([.45,0.,.04]),0.,1.,None),
                    (np.zeros((100,3)),np.array([.50,0.,.04]),0.,2.,None)])
    fake = SimpleNamespace(value=params.__getitem__,observe=lambda **kw:next(samples),
                           publish_observation=lambda *a:None,
                           execute_detected_pose=lambda *a:pytest.fail('must not move'))
    with pytest.raises(RuntimeError, match='Block moved'):
        VisionPickLift.run(fake)


def test_no_camera_data_propagates_failure_without_motion():
    def fail():
        raise RuntimeError('Timed out waiting for two cameras')
    fake = SimpleNamespace(observe=fail, execute_detected_pose=lambda *a:pytest.fail('must not move'))
    with pytest.raises(RuntimeError, match='Timed out'):
        VisionPickLift.run(fake)


def test_measured_pose_is_applied_to_shared_task(monkeypatch):
    captured = []
    class Recorder(VisionPickLift):
        def __init__(self):
            pass
        def value(self, name):
            return {'grasp_yaw_offset': .1}[name]
        def get_logger(self):
            return logger()
        def set_parameters(self, params):
            captured.extend(params)
            return [SimpleNamespace(successful=True)]
    fake = Recorder()
    monkeypatch.setattr(MoveItPickLift,'run',lambda self:captured.append('shared task'))
    VisionPickLift.execute_detected_pose(fake,np.array([.49,-.14,.04]),-.11)
    values = {p.name:p.value for p in captured[:-1]}
    assert values['block_x'] == .49 and values['block_y'] == -.14
    assert values['grasp_yaw'] == pytest.approx(-.01)
    assert values['block_quaternion'][2] == pytest.approx(np.sin(-.11/2))
    assert captured[-1] == 'shared task'


def test_observation_json_contains_detected_pose(tmp_path):
    params = dict(output_directory=str(tmp_path),robot_model='ur3e')
    stamp = SimpleNamespace(to_msg=lambda:__import__('builtin_interfaces.msg',fromlist=['Time']).Time())
    publisher = SimpleNamespace(publish=lambda message:None)
    fake = SimpleNamespace(value=params.__getitem__,get_clock=lambda:SimpleNamespace(now=lambda:stamp),
                           object_pub=publisher,block_pub=publisher,get_logger=logger)
    VisionPickLift.publish_observation(fake,np.zeros((100,3)),np.array([.49,-.14,.04]),-.11)
    data = json.loads((tmp_path/'observation.json').read_text())
    assert data['center'] == [.49,-.14,.04] and data['yaw'] == -.11
    assert data['frame'] == 'world' and data['robot'] == 'ur3e'


@pytest.mark.parametrize('robot', ['fr3','ur3e'])
def test_launch_loads_shared_robot_configuration(monkeypatch,robot):
    from robot_vision_pick_lift import configuration
    original = configuration.share
    monkeypatch.setattr(configuration,'share',lambda name:str(ROOT) if name=='robot_vision_pick_lift' else original(name))
    module = runpy.run_path(str(ROOT/'launch/vision_task.launch.py'))
    context = LaunchContext()
    for action in module['generate_launch_description']().entities[:-1]:
        action.execute(context)
    context.launch_configurations['robot'] = robot
    captured = []
    module['setup'].__globals__['Node'] = lambda **kw:captured.append(kw) or kw
    module['setup'](context)
    params = captured[0]['parameters'][0]
    assert params['robot_model'] == robot
    assert params['planning_group'] == ('fr3_arm' if robot=='fr3' else 'ur_manipulator')
    assert params['perception_only'] is False and params['execute'] is True
    assert params['block_size'] == [.04]*3


@pytest.mark.parametrize('robot', ['fr3','ur3e'])
def test_camera_simulation_reuses_shared_bringup(monkeypatch,robot):
    from robot_vision_pick_lift import configuration
    original = configuration.share
    monkeypatch.setattr(configuration,'share',lambda name:str(ROOT) if name=='robot_vision_pick_lift' else original(name))
    module = runpy.run_path(str(ROOT/'launch/vision_sim.launch.py'))
    context = LaunchContext()
    for action in module['generate_launch_description']().entities[:-1]:
        action.execute(context)
    context.launch_configurations.update(robot=robot,gui='false',seed='17')
    calls, nodes = [], []
    def bringup(arm,gripper,environment,**kwargs):
        calls.append((arm,environment,kwargs))
        return ['shared lifecycle']
    module['setup'].__globals__['create_simulation'] = bringup
    module['setup'].__globals__['Node'] = lambda **kwargs:nodes.append(kwargs) or kwargs
    actions = module['setup'](context)
    assert actions[0] == 'shared lifecycle'
    assert calls[0][0]['model'] == robot and calls[0][2]['gui'] is False
    assert calls[0][2]['world_xml'].count('type="rgbd_camera"') == 2
    assert len([n for n in nodes if n['package']=='tf2_ros']) == 4
    assert len([n for n in nodes if n['package']=='ros_gz_bridge']) == 4
