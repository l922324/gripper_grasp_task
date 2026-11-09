"""Gripper closure must wait for measured arm arrival and standstill."""
from types import SimpleNamespace
import pytest
from ur3e_pick_lift import moveit_pick_lift as module


def run_gate(monkeypatch, samples, timeout=2.):
    clock = [0.]
    seen = []
    values = dict(arm_settle_timeout=timeout, arm_position_tolerance=.005,
                  arm_velocity_tolerance=.01, arm_settle_time=.25)
    node = SimpleNamespace(arm_state=None, arm_state_received=-1., value=values.__getitem__,
                           get_logger=lambda: SimpleNamespace(info=lambda text: seen.append(text)))

    class Executor:
        def add_node(self, node):
            pass
        def spin_once(self, timeout_sec):
            clock[0] += .1
            position, velocity = samples[min(len(seen), len(samples)-1)]
            seen.append((position, velocity))
            message = SimpleNamespace(name=['joint'], position=[position], velocity=[velocity])
            module.MoveItPickLift.receive_arm_state(node, message)
        def remove_node(self, node):
            pass
        def shutdown(self):
            pass

    monkeypatch.setattr(module, 'SingleThreadedExecutor', Executor)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    trajectory = SimpleNamespace(joint_trajectory=SimpleNamespace(
        joint_names=['joint'], points=[SimpleNamespace(positions=[1.])]))
    module.MoveItPickLift.wait_for_arm_settle(node, trajectory)
    return seen, clock[0]


def test_waits_for_both_arrival_and_low_velocity_then_dwell(monkeypatch):
    seen, elapsed = run_gate(monkeypatch, [(0.9, 0.), (1., .2), (1., 0.)])
    assert seen[0] == (0.9, 0.)
    assert seen[1] == (1., .2)
    assert elapsed >= .55
    assert seen[-1] == 'Arm reached target and stopped'


@pytest.mark.parametrize('sample', [(0.9, 0.), (1., .2), (float('nan'), 0.)])
def test_action_success_does_not_bypass_incorrect_or_moving_feedback(monkeypatch, sample):
    with pytest.raises(RuntimeError, match='Arm did not settle'):
        run_gate(monkeypatch, [sample], timeout=.5)


def test_failed_descent_cannot_send_gripper_close():
    commands = []
    values = dict(block_x=.45, block_y=0., block_z=.04, tcp_offset_z=.145,
                  grasp_yaw=0., lift_distance=.1)
    def move_pose(pose, linear):
        commands.append('descent' if linear else 'pregrasp')
        if linear:
            raise RuntimeError('Arm did not settle')
    def move_gripper(position, closing=False, **kwargs):
        commands.append('close' if closing else 'open')
        if not closing:
            assert position == 0.0
        return True
    node = SimpleNamespace(
        add_table=lambda: None, value=values.__getitem__, move_pose=move_pose,
        move_gripper=move_gripper,
        get_logger=lambda: SimpleNamespace(info=lambda text: None))
    with pytest.raises(RuntimeError, match='Arm did not settle'):
        module.MoveItPickLift.run(node)
    assert commands == ['pregrasp', 'open', 'descent']
