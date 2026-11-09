"""Check rejection of hazardous trajectories and preservation of scene rules."""
import math

import pytest
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from ur3e_pick_lift.moveit_pick_lift import validate_trajectory, allow_collision_pair


def trajectory(values):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = ['wrist']
    for i, value in enumerate(values):
        point = JointTrajectoryPoint(positions=[float(value)])
        point.time_from_start.sec = i
        result.joint_trajectory.points.append(point)
    return result


def test_normal_motion():
    assert validate_trajectory(trajectory([0, 0.2, 0.4]), 0.75, 3.14) == pytest.approx(0.4)


@pytest.mark.parametrize('values', [[], [0], [0, math.nan], [0, math.inf], [0, 1.0]])
def test_reject_empty_invalid_or_discontinuous_motion(values):
    with pytest.raises(RuntimeError):
        validate_trajectory(trajectory(values), 0.75, 3.14)


def test_reject_large_detour_even_when_endpoint_is_close():
    with pytest.raises(RuntimeError, match='Excessive travel'):
        validate_trajectory(trajectory([0, 0.5, 1, 1.5, 1, 0.5, 0, -0.5]), 0.75, 3.14)


def test_reject_full_turn_without_wrapping_angles():
    with pytest.raises(RuntimeError, match='Joint jump'):
        validate_trajectory(trajectory([0, 2 * math.pi]), 0.75, 3.14)


def test_reject_bad_timing():
    path = trajectory([0, 0.1])
    path.joint_trajectory.points[1].time_from_start.sec = 0
    with pytest.raises(RuntimeError, match='trajectory times'):
        validate_trajectory(path, 0.75, 3.14)


def test_floor_exception_preserves_existing_collision_rules():
    matrix = AllowedCollisionMatrix(
        entry_names=['base_link', 'wrist', 'finger'],
        entry_values=[AllowedCollisionEntry(enabled=[False, False, False]),
                      AllowedCollisionEntry(enabled=[False, False, True]),
                      AllowedCollisionEntry(enabled=[False, True, False])])
    allow_collision_pair(matrix, 'pick_floor', 'base_link')
    allow_collision_pair(matrix, 'pick_floor', 'base_link_inertia')
    floor = matrix.entry_names.index('pick_floor')
    assert matrix.entry_values[0].enabled[floor]
    assert matrix.entry_values[floor].enabled[0]
    assert not matrix.entry_values[1].enabled[floor]
    assert not matrix.entry_values[2].enabled[floor]
    assert matrix.entry_values[1].enabled[2]
    assert all(len(row.enabled) == len(matrix.entry_names) for row in matrix.entry_values)


def test_pose_goal_is_planned_before_execution():
    from types import SimpleNamespace
    from ur3e_pick_lift.moveit_pick_lift import MoveItPickLift, downward_pose

    captured = []
    executed = []
    path = trajectory([0, 0.1])
    fake = SimpleNamespace(
        check_start=lambda: None,
        value=lambda name: 0.15,
        move=object(),
        action=lambda client, goal: (
            captured.append(goal) or SimpleNamespace(planned_trajectory=path)),
        execute_checked=executed.append)
    MoveItPickLift.move_pose(fake, downward_pose(0.45, 0.0, 0.285, 0.0), keep_down=True)
    assert captured[0].planning_options.plan_only
    constraint = captured[0].request.path_constraints.orientation_constraints[0]
    assert constraint.orientation.w == 0.0
    assert constraint.absolute_x_axis_tolerance == pytest.approx(0.15)
    assert executed == [path]


def test_invalid_plan_never_reaches_execution_action():
    from types import SimpleNamespace
    from ur3e_pick_lift.moveit_pick_lift import MoveItPickLift

    actions = []
    fake = SimpleNamespace(
        value=lambda name: {'max_joint_step': 0.75, 'max_joint_travel': 3.14}[name],
        action=lambda *args: actions.append(args))
    with pytest.raises(RuntimeError, match='Joint jump'):
        MoveItPickLift.execute_checked(fake, trajectory([0, 2 * math.pi]))
    assert not actions
