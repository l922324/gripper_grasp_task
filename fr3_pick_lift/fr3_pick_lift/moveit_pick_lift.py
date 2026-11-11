"""Pose-based FR3 pick/lift using MoveIt's ROS interfaces."""
import copy
import math
import time

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, PositionConstraint, OrientationConstraint,
                             CollisionObject, MoveItErrorCodes, AllowedCollisionEntry,
                             PlanningSceneComponents)
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive, Plane

from .gripper import GripperNode, GRIPPER_OPEN, GRIPPER_CLOSE


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


class MoveItPickLift(GripperNode):
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
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')

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
        # Allow the Gazebo arm and its 50 Hz state stream to settle after contact.
        time.sleep(0.25)
        if linear:
            if not self.cartesian.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('Cartesian planning service unavailable')
            request = GetCartesianPath.Request()
            request.header.frame_id = 'world'
            request.start_state.is_diff = True
            request.group_name = 'fr3_arm'
            request.link_name = 'fr3_link8'
            request.waypoints = [pose]
            request.max_step = 0.002
            # Use an absolute per-joint bound for the redundant seven-joint arm.
            request.jump_threshold = 0.0
            request.revolute_jump_threshold = 0.2
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
        request.group_name = 'fr3_arm'
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
        position.link_name = 'fr3_link8'
        position.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [pose]
        orientation = OrientationConstraint()
        orientation.header.frame_id = 'world'
        orientation.link_name = 'fr3_link8'
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
        # Protect the physical block from an incidental sweep on the first move.
        block = CollisionObject()
        block.header.frame_id = 'world'
        block.id = 'pick_block'
        block.operation = CollisionObject.ADD
        block.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                           dimensions=[0.04, 0.04, 0.04])]
        block_pose = Pose()
        block_pose.position.x = self.value('block_x')
        block_pose.position.y = self.value('block_y')
        block_pose.position.z = self.value('block_z')
        block_pose.orientation.z = math.sin(self.value('grasp_yaw') / 2)
        block_pose.orientation.w = math.cos(self.value('grasp_yaw') / 2)
        block.primitive_poses = [block_pose]
        floor = CollisionObject(id='pick_floor', operation=CollisionObject.ADD)
        floor.header.frame_id = 'world'
        floor.planes = [Plane(coef=[0.0, 0.0, 1.0, 0.0])]
        floor.plane_poses = [Pose()]
        request.scene.world.collision_objects = [table, block, floor]
        if not self.get_scene.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Planning scene query service unavailable')
        query = GetPlanningScene.Request()
        query.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix = self.wait(self.get_scene.call_async(query)).scene.allowed_collision_matrix
        # Preserve all existing exclusions; only the fixed base may touch the floor.
        for name in ('pick_floor', 'fr3_link0'):
            if name not in matrix.entry_names:
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_names.append(name)
                matrix.entry_values.append(AllowedCollisionEntry(
                    enabled=[False] * len(matrix.entry_names)))
        a, b = matrix.entry_names.index('pick_floor'), matrix.entry_names.index('fr3_link0')
        matrix.entry_values[a].enabled[b] = True
        matrix.entry_values[b].enabled[a] = True
        request.scene.allowed_collision_matrix = matrix
        if not self.wait(self.scene.call_async(request)).success:
            raise RuntimeError('Failed to add table to planning scene')

    def allow_grasp_contact(self):
        """Remove the block proxy before the contact phase handled by Gazebo."""
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        block = CollisionObject(id='pick_block', operation=CollisionObject.REMOVE)
        block.header.frame_id = 'world'
        request.scene.world.collision_objects = [block]
        if not self.wait(self.scene.call_async(request)).success:
            raise RuntimeError('Failed to enable block contact')

    def run(self):
        self.add_table()
        # fr3_link8 +Z points down: fr3_link8 sits above the finger grasp point.
        grasp = downward_pose(
            self.value('block_x'), self.value('block_y'),
            self.value('block_z') + self.value('tcp_offset_z'),
            self.value('grasp_yaw'))
        pregrasp = copy.deepcopy(grasp)
        pregrasp.position.z += self.value('lift_distance')

        self.move_pose(pregrasp, False)

        if not self.move_gripper(GRIPPER_OPEN):
            raise RuntimeError('Gripper open failed')

        self.allow_grasp_contact()
        self.move_pose(grasp, True)

        if not self.move_gripper(GRIPPER_CLOSE, closing=True):
            raise RuntimeError('Gripper close failed')
        
        self.move_pose(pregrasp, linear=True)

        time.sleep(self.value('hold_seconds'))

        
        drop_pose = copy.deepcopy(grasp)
        # Leave room for the simulated contact solver at the tabletop.
        drop_pose.position.z += 0.006
        self.move_pose(drop_pose, linear=True)

        if not self.move_gripper(GRIPPER_OPEN):
            raise RuntimeError('Gripper release failed')
        
        self.move_pose(pregrasp, linear=True)

        self.get_logger().info('MoveIt pick/lift sequence complete')


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
