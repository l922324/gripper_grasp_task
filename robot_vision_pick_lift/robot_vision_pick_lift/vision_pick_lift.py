"""Observe two RGB-D views and feed the detected pose into the shared task."""
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from robot_pick_lift.moveit_pick_lift import MoveItPickLift
from .perception import (decode_cloud, green_points, transform_points, crop_table,
                         fuse_object_cloud, estimate_block)


DEFAULTS = {
    'camera_topics': ['/d455_left/points', '/d455_right/points'],
    'observation_timeout': 60., 'max_cloud_age': 5., 'max_camera_skew': 2.,
    'min_camera_points': 30, 'min_object_points': 100, 'voxel_size': 0.002,
    'perception_only': False, 'execute': True, 'output_directory': '/tmp/robot_observation',
    'max_pose_drift': 0.005, 'max_yaw_drift_degrees': 5.,
    'grasp_yaw_offset': 0., 'robot_model': '',
}


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
        for name in ('voxel_size', 'max_cloud_age', 'max_camera_skew', 'observation_timeout',
                     'max_pose_drift', 'max_yaw_drift_degrees'):
            if not np.isfinite(self.value(name)) or self.value(name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.clouds = {}
        self.last_cloud_error = ''
        cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions_ = [self.create_subscription(
            PointCloud2, topic, lambda message, i=i: self.receive(message, i), cloud_qos)
            for i, topic in enumerate(self.value('camera_topics'))]
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.object_pub = self.create_publisher(PointCloud2, '/robot_vision/object_cloud', qos)
        self.block_pub = self.create_publisher(PoseStamped, '/robot_vision/block_pose', qos)

    def table_top(self):
        return self.value('table_position')[2] + self.value('table_size')[2]/2

    def receive(self, message, index):
        try:
            points, colors = decode_cloud(message)
            points = green_points(points, colors)
            raw_green_count = len(points)
            transform = self.tf_buffer.lookup_transform(
                'world', message.header.frame_id,
                rclpy.time.Time.from_msg(message.header.stamp))
            points = crop_table(transform_points(points, transform),
                                self.value('table_position'), self.value('table_size'),
                                self.value('table_quaternion'), self.value('block_size')[2])
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
                    cloud, self.value('block_size'), self.table_top())
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
            f'yaw={np.degrees(yaw):.2f} degrees')
        directory = self.value('output_directory')
        if directory:
            path = Path(directory)
            path.mkdir(parents=True, exist_ok=True)
            np.save(path/'object_cloud.npy', cloud)
            (path/'observation.json').write_text(json.dumps(
                dict(frame='world', robot=self.value('robot_model'), center=center.tolist(),
                     yaw=float(yaw), points=len(cloud)), indent=2))

    def execute_detected_pose(self, center, yaw):
        """Use perception for position, proxy orientation, and grasp alignment."""
        quaternion = Rotation.from_euler('z', yaw).as_quat().tolist()
        parameters = [Parameter('block_' + axis, value=float(value))
                      for axis, value in zip('xyz', center)]
        parameters.extend([
            Parameter('block_quaternion', value=quaternion),
            Parameter('grasp_yaw', value=float(yaw + self.value('grasp_yaw_offset')))])
        results = self.set_parameters(parameters)
        if not all(result.successful for result in results):
            raise RuntimeError('Failed to apply the detected pose to the shared task')
        self.get_logger().info('Detection complete; executing shared pick/lift with the observed pose')
        super().run()

    def run(self):
        cloud, center, yaw, stamp, _ = self.observe()
        self.publish_observation(cloud, center, yaw)
        if self.value('perception_only') or not self.value('execute'):
            return
        # Require another fresh pair before moving; never act on a moving object.
        cloud, current_center, current_yaw, _, _ = self.observe(after=stamp)
        size = self.value('block_size')
        period = math.pi/2 if np.isclose(size[0], size[1]) else math.pi
        error = abs((current_yaw-yaw+period/2) % period-period/2)
        if (np.linalg.norm(current_center-center) > self.value('max_pose_drift') or
                error > math.radians(self.value('max_yaw_drift_degrees'))):
            raise RuntimeError('Block moved between observations; refusing the stale pose')
        self.publish_observation(cloud, current_center, current_yaw)
        self.execute_detected_pose(current_center, current_yaw)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionPickLift()
        node.run()
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
