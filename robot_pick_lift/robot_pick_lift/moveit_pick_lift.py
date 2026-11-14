"""Robot-independent pose-based pick/lift using MoveIt's ROS interfaces."""
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
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene, GetPlanningScene, GetPositionIK
from shape_msgs.msg import SolidPrimitive, Plane

from .gripper import GripperNode
from .geometry import tool_pose_for_grasp, normalized
from .configuration import vector


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
        defaults = {
            'planning_group': '', 'tool_link': '',
            'floor_contact_links': [''],
            'reference_frame': 'world',
            'table_size': [0.60, 0.80, 0.04], 'table_position': [0.45, 0.0, 0.0],
            'table_quaternion': [0.0, 0.0, 0.0, 1.0],
            'block_size': [0.04, 0.04, 0.04],
            'block_quaternion': [0.0, 0.0, 0.0, 1.0], 'floor_z': 0.0,
            'ik_service': '/compute_ik', 'ik_timeout': 2.0,
            'block_x': 0.45, 'block_y': 0.0, 'block_z': 0.04,
            'tool_to_grasp_xyz': [0.0, 0.0, 0.145],
            'tool_to_grasp_quaternion': [0.0, 0.0, 0.0, 1.0],
            'grasp_yaw': 0.0, 'lift_distance': 0.10, 'hold_seconds': 3.0,
            'settle_seconds': 0.25, 'release_clearance': 0.006,
            'jump_threshold': 0.0, 'revolute_jump_threshold': 0.2,
            'move_action': '/move_action', 'execute_action': '/execute_trajectory',
            'cartesian_service': '/compute_cartesian_path',
            'apply_scene_service': '/apply_planning_scene',
            'get_scene_service': '/get_planning_scene',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, self.profile_defaults.get(name, value))
        for name in ('planning_group', 'tool_link'):
            if not self.value(name):
                raise ValueError(f'Missing robot profile parameter: {name}')
        if not all(self.value('floor_contact_links')):
            raise ValueError('Missing robot profile parameter: floor_contact_links')
        for name in ('lift_distance', 'hold_seconds', 'settle_seconds',
                     'release_clearance', 'jump_threshold', 'revolute_jump_threshold'):
            value = self.value(name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        for name in ('block_x', 'block_y', 'block_z', 'grasp_yaw',
                     'gripper_open', 'gripper_close', 'gripper_max_effort'):
            if not math.isfinite(self.value(name)):
                raise ValueError(f'{name} must be finite')
        if self.value('gripper_timeout') <= 0 or not math.isfinite(self.value('gripper_timeout')):
            raise ValueError('gripper_timeout must be finite and positive')
        for name in ('table_size', 'block_size'):
            vector(self.value(name), 3, name, positive=True)
        vector(self.value('table_position'), 3, 'table_position')
        normalized(self.value('table_quaternion'))
        normalized(self.value('block_quaternion'))
        if not math.isfinite(self.value('floor_z')):
            raise ValueError('floor_z must be finite')
        if not math.isfinite(self.value('ik_timeout')) or self.value('ik_timeout') <= 0:
            raise ValueError('ik_timeout must be finite and positive')
        tool_pose_for_grasp([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0],
                            self.value('tool_to_grasp_xyz'),
                            self.value('tool_to_grasp_quaternion'))
        self.move = ActionClient(self, MoveGroup, self.value('move_action'))
        self.execute = ActionClient(self, ExecuteTrajectory, self.value('execute_action'))
        self.cartesian = self.create_client(GetCartesianPath, self.value('cartesian_service'))
        self.scene = self.create_client(ApplyPlanningScene, self.value('apply_scene_service'))
        self.ik = self.create_client(GetPositionIK, self.value('ik_service'))
        self.get_scene = self.create_client(GetPlanningScene, self.value('get_scene_service'))

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
        time.sleep(self.value('settle_seconds'))
        if linear:
            if not self.cartesian.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('Cartesian planning service unavailable')
            request = GetCartesianPath.Request()
            request.header.frame_id = self.value('reference_frame')
            request.start_state.is_diff = True
            request.group_name = self.value('planning_group')
            request.link_name = self.value('tool_link')
            request.waypoints = [pose]
            request.max_step = 0.002
            # Joint-jump policy is supplied by the robot profile.
            request.jump_threshold = self.value('jump_threshold')
            request.revolute_jump_threshold = self.value('revolute_jump_threshold')
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
        request.group_name = self.value('planning_group')
        request.pipeline_id = 'ompl'
        request.planner_id = 'RRTConnectkConfigDefault'
        request.num_planning_attempts = 10
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        request.start_state.is_diff = True
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.value('reference_frame')
        position.link_name = self.value('tool_link')
        position.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [pose]
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.value('reference_frame')
        orientation.link_name = self.value('tool_link')
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
        try:
            self.action(self.move, goal)
        except RuntimeError as exc:
            p = pose.position
            raise RuntimeError(f'No valid pose plan/execution at '
                               f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}): {exc}') from exc

    def add_table(self):
        if not self.scene.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Planning scene service unavailable')
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        table = CollisionObject()
        table.header.frame_id = self.value('reference_frame')
        table.id = 'pick_table'
        table.operation = CollisionObject.ADD
        table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                           dimensions=self.value('table_size'))]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.value('table_position')
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = normalized(self.value('table_quaternion'))
        table.primitive_poses = [pose]
        # Protect the physical block from an incidental sweep on the first move.
        block = CollisionObject()
        block.header.frame_id = self.value('reference_frame')
        block.id = 'pick_block'
        block.operation = CollisionObject.ADD
        block.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                           dimensions=self.value('block_size'))]
        block_pose = Pose()
        block_pose.position.x = self.value('block_x')
        block_pose.position.y = self.value('block_y')
        block_pose.position.z = self.value('block_z')
        (block_pose.orientation.x, block_pose.orientation.y,
         block_pose.orientation.z, block_pose.orientation.w) = normalized(self.value('block_quaternion'))
        block.primitive_poses = [block_pose]
        floor = CollisionObject(id='pick_floor', operation=CollisionObject.ADD)
        floor.header.frame_id = self.value('reference_frame')
        floor.planes = [Plane(coef=[0.0, 0.0, 1.0, -self.value('floor_z')])]
        floor_pose = Pose()
        floor_pose.orientation.w = 1.0
        floor.plane_poses = [floor_pose]
        self.block_proxy = copy.deepcopy(block)
        request.scene.world.collision_objects = [table, block, floor]
        if not self.get_scene.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Planning scene query service unavailable')
        query = GetPlanningScene.Request()
        query.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix = self.wait(self.get_scene.call_async(query)).scene.allowed_collision_matrix
        # Preserve existing exclusions and allow only mounting links at the floor.
        contacts = self.value('floor_contact_links')
        for name in ['pick_floor'] + contacts:
            if name not in matrix.entry_names:
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_names.append(name)
                matrix.entry_values.append(AllowedCollisionEntry(
                    enabled=[False] * len(matrix.entry_names)))
        floor_index = matrix.entry_names.index('pick_floor')
        for name in contacts:
            link_index = matrix.entry_names.index(name)
            matrix.entry_values[floor_index].enabled[link_index] = True
            matrix.entry_values[link_index].enabled[floor_index] = True
        request.scene.allowed_collision_matrix = matrix
        if not self.wait(self.scene.call_async(request)).success:
            raise RuntimeError('Failed to add table to planning scene')

    def allow_grasp_contact(self):
        """Remove the block proxy before the contact phase handled by Gazebo."""
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        block = CollisionObject(id='pick_block', operation=CollisionObject.REMOVE)
        block.header.frame_id = self.value('reference_frame')
        request.scene.world.collision_objects = [block]
        if not self.wait(self.scene.call_async(request)).success:
            raise RuntimeError('Failed to enable block contact')

    def check_targets(self, targets):
        """Require collision-free IK for every target before sending arm/gripper goals.

        Open-finger state is used for contact poses. The temporary proxy is
        removed during these checks and restored before the initial arm plan.
        """
        if not self.ik.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('IK service unavailable; start MoveIt before the task')
        query = GetPlanningScene.Request()
        query.components.components = PlanningSceneComponents.ROBOT_STATE
        state = copy.deepcopy(self.wait(self.get_scene.call_async(query)).scene.robot_state)
        joint = self.value('gripper_joint')
        if joint not in state.joint_state.name:
            raise RuntimeError(f'Gripper joint missing from current robot state: {joint}')
        index = state.joint_state.name.index(joint)
        state.joint_state.position[index] = self.value('gripper_open')
        # Recompute mimic joints from the commanded driver joint in MoveIt.
        state.is_diff = True
        self.allow_grasp_contact()
        try:
            for phase, pose in targets:
                request = GetPositionIK.Request()
                ik = request.ik_request
                ik.group_name = self.value('planning_group')
                ik.ik_link_name = self.value('tool_link')
                ik.robot_state = copy.deepcopy(state)
                ik.pose_stamped.header.frame_id = self.value('reference_frame')
                ik.pose_stamped.pose = pose
                ik.avoid_collisions = True
                timeout = self.value('ik_timeout')
                ik.timeout.sec = int(timeout)
                ik.timeout.nanosec = int((timeout - int(timeout)) * 1e9)
                result = self.wait(self.ik.call_async(request), timeout + 10.0)
                if result.error_code.val != MoveItErrorCodes.SUCCESS:
                    p = pose.position
                    raise RuntimeError(
                        f'{phase}: no collision-free IK found at '
                        f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}); '
                        f'target may be unreachable or in collision '
                        f'(MoveIt code {result.error_code.val})')
                state = result.solution
        finally:
            restore = ApplyPlanningScene.Request()
            restore.scene.is_diff = True
            restore.scene.robot_state.is_diff = True
            restore.scene.world.collision_objects = [copy.deepcopy(self.block_proxy)]
            if not self.wait(self.scene.call_async(restore)).success:
                raise RuntimeError('Failed to restore block collision proxy after preflight')

    def run(self):
        self.add_table()
        target = downward_pose(
            self.value('block_x'), self.value('block_y'),
            self.value('block_z'), self.value('grasp_yaw'))
        xyz, quaternion = tool_pose_for_grasp(
            [target.position.x, target.position.y, target.position.z],
            [target.orientation.x, target.orientation.y,
             target.orientation.z, target.orientation.w],
            self.value('tool_to_grasp_xyz'),
            self.value('tool_to_grasp_quaternion'))
        grasp = Pose()
        grasp.position.x, grasp.position.y, grasp.position.z = xyz
        (grasp.orientation.x, grasp.orientation.y,
         grasp.orientation.z, grasp.orientation.w) = quaternion
        pregrasp = copy.deepcopy(grasp)
        pregrasp.position.z += self.value('lift_distance')

        drop_pose = copy.deepcopy(grasp)
        drop_pose.position.z += self.value('release_clearance')
        self.check_targets([('PREGRASP', pregrasp), ('GRASP', grasp),
                            ('LIFT', pregrasp), ('RELEASE', drop_pose)])
        self.move_pose(pregrasp, False)

        if not self.move_gripper(self.value('gripper_open')):
            raise RuntimeError('Gripper open failed')

        self.allow_grasp_contact()
        self.move_pose(grasp, True)

        if not self.move_gripper(self.value('gripper_close'), closing=True):
            raise RuntimeError('Gripper close failed')
        
        self.move_pose(pregrasp, linear=True)

        time.sleep(self.value('hold_seconds'))

        
        self.move_pose(drop_pose, linear=True)

        if not self.move_gripper(self.value('gripper_open')):
            raise RuntimeError('Gripper release failed')
        
        self.move_pose(pregrasp, linear=True)

        self.get_logger().info('MoveIt pick/lift sequence complete')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MoveItPickLift()
        node.run()
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
