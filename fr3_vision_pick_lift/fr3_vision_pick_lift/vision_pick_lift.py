"""Observe both cameras, request GraspGenX grasps, and execute a feasible top-down grasp."""
import copy
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from sensor_msgs.msg import PointCloud2, PointField
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from fr3_pick_lift.moveit_pick_lift import MoveItPickLift
from fr3_pick_lift.gripper import GRIPPER_OPEN, GRIPPER_CLOSE
from .inference import infer_grasps
from .perception import (decode_cloud, green_points, transform_points, crop_table,
                         fuse_object_cloud, estimate_block)
from .selection import select_candidates, mount_matrix


DEFAULTS = {
    'camera_topics': ['/d455_left/points', '/d455_right/points'],
    'observation_timeout': 60., 'max_cloud_age': 5., 'max_camera_skew': 2.,
    'min_camera_points': 30, 'min_object_points': 100, 'voxel_size': 0.002,
    'block_size': 0.04, 'table_top': 0.02,
    'graspgen_endpoint': 'tcp://127.0.0.1:5556', 'inference_timeout': 60.,
    'num_grasps': 512, 'max_candidates': 20, 'minimum_score': 0.5,
    'max_tilt_degrees': 5., 'model_base_to_flange_xyz': [0., 0., 0.],
    'model_base_to_flange_rpy': [0., 0., 0.], 'gripper_tip_depth': 0.163,
    'perception_only': False, 'execute': True, 'output_directory': '',
}


def matrix_pose(matrix):
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = map(float, q)
    return result


def cloud_message(points, stamp):
    data = np.zeros(len(points), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    for index, name in enumerate(('x', 'y', 'z')):
        data[name] = points[:, index]
    data['rgb'] = 0x00CC00
    message = PointCloud2()
    message.header.frame_id, message.header.stamp = 'world', stamp
    message.height, message.width = 1, len(points)
    message.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                      for index, name in enumerate(('x', 'y', 'z'))]
    message.fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
    message.point_step, message.row_step = 16, 16*len(points)
    message.is_dense, message.is_bigendian = True, False
    message.data = data.tobytes()
    return message


class VisionPickLift(MoveItPickLift):
    def __init__(self):
        super().__init__()
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        if len(self.value('camera_topics')) != 2:
            raise ValueError('Exactly two camera topics are required')
        for name in ('voxel_size', 'max_cloud_age', 'observation_timeout', 'inference_timeout'):
            if not np.isfinite(self.value(name)) or self.value(name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.clouds = {}
        self.last_cloud_error = ''
        # Gazebo bridges publish reliably. Match them so fragmented RGB-D clouds
        # can be retransmitted instead of disappearing on the VM's transport.
        cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions_ = [self.create_subscription(
            PointCloud2, topic, lambda message, i=i: self.receive(message, i),
            cloud_qos) for i, topic in enumerate(self.value('camera_topics'))]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.object_pub = self.create_publisher(PointCloud2, '/fr3_vision/object_cloud', qos)
        self.block_pub = self.create_publisher(PoseStamped, '/fr3_vision/block_pose', qos)
        self.candidates_pub = self.create_publisher(PoseArray, '/fr3_vision/grasp_candidates', qos)
        self.grasp_pub = self.create_publisher(PoseStamped, '/fr3_vision/selected_grasp', qos)
        self.block_center = None
        self.block_yaw = 0.
        self.pose_plan = None

    def receive(self, message, index):
        try:
            points, colors = decode_cloud(message)
            points = green_points(points, colors)
            raw_green_count = len(points)
            transform = self.tf_buffer.lookup_transform(
                'world', message.header.frame_id,
                rclpy.time.Time.from_msg(message.header.stamp))
            points = crop_table(transform_points(points, transform), self.value('table_top'))
            if len(points) < self.value('min_camera_points'):
                raise ValueError(f'Camera {index}: only {len(points)} green object points in the table crop '
                                 f'(raw green={raw_green_count}, frame={message.header.frame_id})')
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self.clouds[index] = (stamp, points)
        except Exception as exc:
            self.last_cloud_error = str(exc)

    def observe(self, after=-1.):
        # Keep the executor attached across spins: repeatedly adding/removing the
        # node can starve cloud callbacks behind Fortress's high-rate /clock.
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            return self._observe(executor, after)
        finally:
            executor.remove_node(self)
            executor.shutdown()

    def _observe(self, executor, after):
        deadline = time.monotonic() + self.value('observation_timeout')
        last_error = ''
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            if len(self.clouds) != 2:
                continue
            samples = [self.clouds[i] for i in (0, 1)]
            stamps = [sample[0] for sample in samples]
            now = self.get_clock().now().nanoseconds * 1e-9
            ages = [now - stamp for stamp in stamps]
            if (min(stamps) <= after or min(ages) < -0.25 or
                    max(ages) > self.value('max_cloud_age') or
                    max(stamps)-min(stamps) > self.value('max_camera_skew')):
                last_error = f'Cloud ages={ages}, timestamp skew={max(stamps)-min(stamps):.3f}s'
                continue
            try:
                cloud = fuse_object_cloud([sample[1] for sample in samples],
                                          self.value('voxel_size'), self.value('min_object_points'))
                center, yaw, extents = estimate_block(
                    cloud, self.value('block_size'), self.value('table_top'))
                return cloud, center, yaw, max(stamps), extents
            except Exception as exc:
                last_error = str(exc)
        raise RuntimeError('Timed out waiting for two fresh, consistent RGB-D observations. '
                           + (last_error or self.last_cloud_error))

    def publish_observation(self, cloud, center, yaw):
        stamp = self.get_clock().now().to_msg()
        self.object_pub.publish(cloud_message(cloud, stamp))
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'world', stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, center)
        q = Rotation.from_euler('z', yaw).as_quat()
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q)
        self.block_pub.publish(pose)
        self.get_logger().info(
            f'Observed {len(cloud)} fused points; block center={center.tolist()}, '
            f'yaw modulo 90 degrees={np.degrees(yaw):.2f} degrees')
        directory = self.value('output_directory')
        if directory:
            path = Path(directory)
            path.mkdir(parents=True, exist_ok=True)
            np.save(path/'object_cloud.npy', cloud)
            (path/'observation.json').write_text(json.dumps(
                dict(center=center.tolist(), yaw=float(yaw), points=len(cloud)), indent=2))

    def block_proxy(self):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = request.scene.robot_state.is_diff = True
        block = CollisionObject(id='pick_block', operation=CollisionObject.ADD)
        block.header.frame_id = 'world'
        block.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                           dimensions=[self.value('block_size')]*3)]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, self.block_center)
        pose.orientation.z, pose.orientation.w = float(np.sin(self.block_yaw/2)), float(np.cos(self.block_yaw/2))
        block.primitive_poses = [pose]
        request.scene.world.collision_objects = [block]
        if not self.wait(self.scene.call_async(request)).success:
            raise RuntimeError('Failed to update detected block collision proxy')

    def action(self, client, goal):
        # Pose planning through the inherited method always previews first in this node.
        # Execution happens explicitly, after the Cartesian approach was also checked.
        if client is self.move:
            goal.planning_options.plan_only = True
            self.pose_plan = super().action(client, goal)
            return self.pose_plan
        return super().action(client, goal)

    def check_approach(self, plan, target):
        state = copy.deepcopy(plan.trajectory_start)
        joints = plan.planned_trajectory.joint_trajectory
        if not joints.points:
            raise RuntimeError('MoveIt returned an empty pregrasp plan')
        values = dict(zip(state.joint_state.name, state.joint_state.position))
        values.update(zip(joints.joint_names, joints.points[-1].positions))
        state.joint_state.name = list(values)
        state.joint_state.position = list(values.values())
        state.joint_state.velocity, state.joint_state.effort = [], []
        state.is_diff = False
        if not self.cartesian.wait_for_service(timeout_sec=10.):
            raise RuntimeError('Cartesian planning service unavailable')
        request = GetCartesianPath.Request()
        request.header.frame_id = 'world'
        request.start_state = state
        request.group_name, request.link_name = 'fr3_arm', 'fr3_link8'
        request.waypoints = [target]
        request.max_step, request.revolute_jump_threshold = 0.002, 0.2
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = 0.1
        self.allow_grasp_contact()
        try:
            result = self.wait(self.cartesian.call_async(request))
        finally:
            self.block_proxy()
        if result.error_code.val != 1 or result.fraction < 0.999:
            raise RuntimeError(f'Approach fraction={result.fraction:.3f}, error={result.error_code.val}')

    def run(self):
        cloud, center, yaw, stamp, _ = self.observe()
        self.publish_observation(cloud, center, yaw)
        if self.value('perception_only'):
            return
        grasps, scores = infer_grasps(cloud, self.value('graspgen_endpoint'),
                                     self.value('inference_timeout'), self.value('num_grasps'))
        if self.value('output_directory'):
            np.savez(Path(self.value('output_directory'))/'graspgen_candidates.npz',
                     grasps=grasps, scores=scores)
        # Inference can take time; process new sensor frames and reject scene changes.
        _, current_center, current_yaw, fresh_stamp, _ = self.observe(after=stamp)
        if (np.linalg.norm(current_center-center) > 0.005 or
                abs((current_yaw-yaw+np.pi/4) % (np.pi/2)-np.pi/4) > np.radians(5.)):
            raise RuntimeError('Block moved during inference; capture a new cloud and retry')
        self.block_center, self.block_yaw = center, yaw
        self.set_parameters([Parameter('block_' + axis, value=float(value))
                             for axis, value in zip('xyz', center)])
        candidates = select_candidates(
            grasps, scores, center, self.value('minimum_score'), self.value('max_tilt_degrees'),
            mount_matrix(self.value('model_base_to_flange_xyz'), self.value('model_base_to_flange_rpy')),
            self.value('gripper_tip_depth'), self.value('table_top'), self.value('tcp_offset_z'))
        # The calibrated close target is for opposing faces of a 40 mm cube.
        # Reject diagonal grasps requiring a wider opening/contact calibration.
        candidates = [(score, matrix) for score, matrix in candidates
                      if abs((np.arctan2(matrix[1, 0], matrix[0, 0]) - yaw + np.pi/4)
                             % (np.pi/2) - np.pi/4) <= np.radians(8.)]
        message = PoseArray()
        message.header.frame_id, message.header.stamp = 'world', self.get_clock().now().to_msg()
        message.poses = [matrix_pose(matrix) for _, matrix in candidates]
        self.candidates_pub.publish(message)
        if not candidates:
            raise RuntimeError('No scored top-down grasp clears the table and matches the object. '
                               'Verify the GraspGenX gripper-base calibration; no pose was substituted.')
        self.add_table()
        self.block_proxy()
        if self.value('execute') and not self.move_gripper(GRIPPER_OPEN):
            raise RuntimeError('Gripper open failed')
        selected = None
        for score, matrix in candidates[:self.value('max_candidates')]:
            grasp = matrix_pose(matrix)
            pregrasp = copy.deepcopy(grasp)
            pregrasp.position.z += self.value('lift_distance')
            try:
                self.move_pose(pregrasp)  # plan-only via action override
                self.check_approach(self.pose_plan, grasp)
                selected = score, grasp, pregrasp, self.pose_plan
                break
            except RuntimeError as exc:
                self.get_logger().info(f'Rejected candidate score={score:.3f}: {exc}')
        if selected is None:
            raise RuntimeError('No GraspGenX candidate has a feasible pregrasp and full approach')
        score, grasp, pregrasp, plan = selected
        selected_msg = PoseStamped(header=message.header, pose=grasp)
        self.grasp_pub.publish(selected_msg)
        if self.value('output_directory'):
            (Path(self.value('output_directory'))/'selected_grasp.json').write_text(json.dumps(
                dict(frame='world', link='fr3_link8', score=score,
                     position=[grasp.position.x, grasp.position.y, grasp.position.z],
                     quaternion_xyzw=[grasp.orientation.x, grasp.orientation.y,
                                      grasp.orientation.z, grasp.orientation.w]), indent=2))
        self.get_logger().info(f'Selected highest-scoring feasible top-down grasp: {score:.4f}')
        if not self.value('execute'):
            return
        _, check_center, check_yaw, _, _ = self.observe(after=fresh_stamp)
        if (np.linalg.norm(check_center-center) > 0.005 or
                abs((check_yaw-yaw+np.pi/4) % (np.pi/2)-np.pi/4) > np.radians(5.)):
            raise RuntimeError('Block moved while planning; refusing the stale grasp')
        goal = ExecuteTrajectory.Goal(trajectory=plan.planned_trajectory)
        self.action(self.execute, goal)
        self.allow_grasp_contact()
        self.move_pose(grasp, linear=True)
        if not self.move_gripper(GRIPPER_CLOSE, closing=True):
            raise RuntimeError('Gripper close failed')
        self.move_pose(pregrasp, linear=True)
        time.sleep(self.value('hold_seconds'))
        release = copy.deepcopy(grasp)
        release.position.z += 0.006
        self.move_pose(release, linear=True)
        if not self.move_gripper(GRIPPER_OPEN):
            raise RuntimeError('Gripper release failed')
        self.move_pose(pregrasp, linear=True)
        self.get_logger().info('Vision-guided pick/lift motion sequence complete')


def main(args=None):
    rclpy.init(args=args)
    node = VisionPickLift()
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
