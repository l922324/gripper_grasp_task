"""Compose shared arm/gripper/environment settings with camera configuration."""
import math
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory as share
import numpy as np
from robot_pick_lift.configuration import load_configuration, read_yaml, vector


def load_vision_configuration(robot='fr3', description_package='', arm_config='',
                              gripper_config='', environment_config='', vision_config=''):
    own = Path(share('robot_vision_pick_lift'))
    environment_config = environment_config or str(own / 'config/environment.yaml')
    arm, gripper, env, task = load_configuration(
        robot, description_package, arm_config, gripper_config, environment_config)
    vision = read_yaml(vision_config or own / 'config/vision.yaml')
    validate_vision(env, vision)
    params = {**task, **vision['parameters'], 'use_sim_time': True,
              'camera_topics': ['/' + camera['name'] + '/points' for camera in vision['cameras']],
              'grasp_yaw_offset': float(env['task']['grasp_yaw_offset'])}
    return arm, gripper, env, vision, params


def validate_vision(env, vision):
    # This task estimates a resting block and executes world-vertical grasps.
    if not np.allclose(env['table']['orientation_rpy'][:2], [0, 0]):
        raise ValueError('Vision top-down picking requires a horizontal table (roll/pitch zero)')
    if not np.allclose(env['block']['orientation_rpy'][:2], [0, 0]):
        raise ValueError('Vision pose estimation requires a block resting flat on the table')
    cameras = vision['cameras']
    if len(cameras) != 2 or len({c['name'] for c in cameras}) != 2:
        raise ValueError('Exactly two cameras with unique names are required')
    for camera in cameras:
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', camera['name']):
            raise ValueError('Camera name must be a simple frame/topic identifier')
        position = vector(camera['position'], 3, 'camera.position')
        target = vector(camera['target'], 3, 'camera.target')
        if np.linalg.norm(np.subtract(target, position)) < 0.1:
            raise ValueError('Camera position and target must be distinct')
    for key in ('camera_width', 'camera_height'):
        if not isinstance(vision[key], int) or isinstance(vision[key], bool) or vision[key] <= 0:
            raise ValueError(f'{key} must be a positive integer')
    for key in ('camera_rate', 'near_clip', 'far_clip', 'horizontal_fov'):
        if not math.isfinite(vision[key]) or vision[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    if vision['near_clip'] >= vision['far_clip'] or vision['horizontal_fov'] >= math.pi:
        raise ValueError('Invalid camera clipping range or field of view')
    random = vision['randomization']
    if not isinstance(random['enabled'], bool) or not isinstance(random['seed'], int):
        raise ValueError('randomization.enabled must be boolean and seed must be integer')
    for key in ('x_range', 'y_range', 'yaw_range'):
        bounds = vector(random[key], 2, key)
        if bounds[0] > bounds[1]:
            raise ValueError(f'{key} must be ordered')
    if random['enabled']:
        radius = math.hypot(*env['block']['size'][:2]) / 2
        for key, length in zip(('x_range', 'y_range'), env['table']['size'][:2]):
            lo, hi = random[key]
            if lo - radius <= -length/2 or hi + radius >= length/2:
                raise ValueError(f'{key} must keep rotated block corners inside the configured table')
    # Validate early so a bad perception config cannot start a simulation.
    p = vision['parameters']
    for key in ('observation_timeout', 'max_cloud_age', 'max_camera_skew', 'voxel_size',
                'max_pose_drift', 'max_yaw_drift_degrees'):
        if not math.isfinite(p[key]) or p[key] <= 0:
            raise ValueError(f'{key} must be finite and positive')
    for key in ('min_camera_points', 'min_object_points'):
        if not isinstance(p[key], int) or p[key] <= 0:
            raise ValueError(f'{key} must be a positive integer')
    for key in ('perception_only', 'execute'):
        if not isinstance(p[key], bool):
            raise ValueError(f'{key} must be boolean')
    return vision
