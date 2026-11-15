"""A missing inference backend must not start the motion sequence."""
from types import SimpleNamespace
import numpy as np
import pytest
from fr3_vision_pick_lift import vision_pick_lift as module


def test_inference_failure_stops_before_any_controller_or_scene_command(monkeypatch):
    commands = []
    fake = SimpleNamespace(
        observe=lambda: (np.zeros((100,3)), np.array([.45,0,.04]), 0., 1., [.04,.04]),
        publish_observation=lambda *args: None,
        value={'perception_only':False, 'graspgen_endpoint':'tcp://missing:5556',
               'inference_timeout':.1, 'num_grasps':100}.__getitem__,
        add_table=lambda: commands.append('scene'),
        move_gripper=lambda *args: commands.append('gripper'),
        move_pose=lambda *args: commands.append('arm'))

    def unavailable(*args):
        raise RuntimeError('server unavailable')

    monkeypatch.setattr(module, 'infer_grasps', unavailable)
    with pytest.raises(RuntimeError, match='server unavailable'):
        module.VisionPickLift.run(fake)
    assert commands == []
