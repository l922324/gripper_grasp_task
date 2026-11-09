"""Pose-based UR3e pick/lift using MoveIt's ROS interfaces."""
import copy
import math
import time

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, PositionConstraint, OrientationConstraint,
                             CollisionObject, MoveItErrorCodes)
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from .fixed_pick_lift import FixedPickLift, GRIPPER_OPEN, GRIPPER_CLOSE


def downward_pose(x, y, z, yaw):
    """Construct Rz(yaw) * Rx(pi), explicitly overriding ROS defaults."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = math.cos(yaw / 2)
    pose.orientation.y = math.sin(yaw / 2)
    pose.orientation.z = 0.0
    pose.orientation.w = 0.0
    return pose


class MoveItPickLift(FixedPickLift):
    """Reuse the existing gripper handling; plan all arm moves with MoveIt."""

    def __init__(self):
        super().__init__()
        for name, value in [('block_x', 0.45), ('block_y', 0.0),
                            ('block_z', 0.04), ('tcp_offset_z', 0.145),
                            ('grasp_yaw', 0.0), ('lift_distance', 0.10),
                            ('hold_seconds', 3.0)]:
            self.declare_parameter(name, value)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def value(self, name):
        return self.get_parameter(name).value

    def wait(self, future, timeout=60.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('MoveIt request timed out')
        return future.result()

    def action(self, client, goal):
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('MoveIt action server unavailable; start moveit.launch.py')
        handle = self.wait(client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError('MoveIt rejected the goal')
        future = handle.get_result_async()
        try:
            wrapped = self.wait(future)
        except RuntimeError:
            self.wait(handle.cancel_goal_async(), 5.0)
            raise
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS):
            raise RuntimeError(f'MoveIt failed: {wrapped.result.error_code.val}')
        return wrapped.result

    def move_pose(self, pose, linear=False):
        if linear:
            if not self.cartesian.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('Cartesian planning service unavailable')
            request = GetCartesianPath.Request()
            request.header.frame_id = 'world'
            request.start_state.is_diff = True
            request.group_name = 'ur_manipulator'
            request.link_name = 'tool0'
            request.waypoints = [pose]
            request.max_step = 0.002
            request.jump_threshold = 2.0
            request.avoid_collisions = True
            request.max_velocity_scaling_factor = 0.1
            request.max_acceleration_scaling_factor = 0.1
            result = self.wait(self.cartesian.call_async(request))
            if result.error_code.val != 1 or result.fraction < 0.999:
                raise RuntimeError(f'Incomplete Cartesian path: {result.fraction:.3f}')
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = result.solution
            self.action(self.execute, goal)
            return
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'ur_manipulator'
        request.pipeline_id = 'ompl'
        request.planner_id = 'RRTConnectkConfigDefault'
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        request.start_state.is_diff = True
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = 'world'
        position.link_name = 'tool0'
        position.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [pose]
        orientation = OrientationConstraint()
        orientation.header.frame_id = 'world'
        orientation.link_name = 'tool0'
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        request.goal_constraints = [constraints]
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        self.action(self.move, goal)

    def add_table(self):
        if not self.scene.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Planning scene service unavailable')
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        table = CollisionObject()
        table.header.frame_id = 'world'
        table.id = 'pick_table'
        table.operation = CollisionObject.ADD
        table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                           dimensions=[0.60, 0.80, 0.04])]
        pose = Pose()
        pose.position.x = 0.45
        pose.orientation.w = 1.0
        table.primitive_poses = [pose]
        request.scene.world.collision_objects = [table]
        if not self.wait(self.scene.call_async(request)).success:
            raise RuntimeError('Failed to add table to planning scene')

    def run(self):
        self.add_table()
        # tool0 +Z points down: tool0 sits above the finger grasp point.
        grasp = downward_pose(
            self.value('block_x'), self.value('block_y'),
            self.value('block_z') + self.value('tcp_offset_z'),
            self.value('grasp_yaw'))
        pregrasp = copy.deepcopy(grasp)
        pregrasp.position.z += self.value('lift_distance')


        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Starting MOVEIT PICK + LIFT primitive'
        )

        self.get_logger().info(
            '========================================'
        )

        # 1. PRE-GRASP
        
        self.move_pose(pregrasp, False)

        # 2. OPEN

        if not self.move_gripper(GRIPPER_OPEN):
            raise RuntimeError('GRIPPER_OPEN_FAILED')

        # 3. APPROACH / GRASP
        
        self.move_pose(grasp, True)

        # 4. CLOSE
        
        if not self.move_gripper(
            GRIPPER_CLOSE,
            closing=True
        ):
            raise RuntimeError('GRIPPER_CLOSE_FAILED')

        # 5. LIFT
        #
        # PRE_GRASP is deliberately 100 mm above GRASP.

        self.move_pose(pregrasp, linear=True)

        # 7. HOLD
        
        self.get_logger().info(
            'Holding object for 3 seconds...'
        )

        time.sleep(self.value('hold_seconds'))

        # 8. RELEASE / APPROACH
        
        drop_pose = copy.deepcopy(grasp)
        drop_pose.position.z += 0.001
        self.move_pose(drop_pose, linear=True)

        # 9. OPEN

        if not self.move_gripper(GRIPPER_OPEN):
            raise RuntimeError('GRIPPER_OPEN_FAILED')

        # 10. RELEASE / APPROACH
        
        self.move_pose(pregrasp, linear=True)

        # PASS
        
        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'MOVEIT PICKUP SIM PASS'
        )

        self.get_logger().info(
            '========================================'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MoveItPickLift()
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
