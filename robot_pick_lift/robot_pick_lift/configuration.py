"""Shared configuration contract for simulation and task geometry."""
from pathlib import Path
import math
import re

from ament_index_python.packages import get_package_share_directory as share
import yaml

from .geometry import quaternion_from_rpy, normalized, multiply


def read_yaml(path):
    with open(path) as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'Expected a YAML mapping: {path}')
    return data


def vector(value, length, name, positive=False):
    if (not isinstance(value, (list, tuple)) or len(value) != length
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) or (positive and v <= 0) for v in value)):
        raise ValueError(f'{name} must contain {length} finite'
                         + (' positive' if positive else '') + ' numbers')
    return [float(v) for v in value]


def resource(spec):
    """Resolve a package-relative resource, or an explicit absolute path."""
    if isinstance(spec, str):
        path = Path(spec)
        if not path.is_absolute():
            raise ValueError(f'Resource path must be absolute: {path}')
    else:
        path = Path(share(spec['package'])) / spec['path']
    if not path.is_file():
        raise ValueError(f'Resource does not exist: {path}')
    return path


def validate_environment(env):
    if env.get('reference_frame') != 'world':
        raise ValueError('Generated Gazebo environments require reference_frame: world')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', env['world_name']):
        raise ValueError('world_name must be a simple Gazebo identifier')
    for name in ('table', 'block'):
        item = env[name]
        for key in ('size', 'position', 'orientation_rpy'):
            item[key] = vector(item[key], 3, f'{name}.{key}', positive=key == 'size')
        item['color'] = vector(item['color'], 4, f'{name}.color')
        if any(v < 0 or v > 1 for v in item['color']):
            raise ValueError(f'{name}.color must be between 0 and 1')
    for name, number in [('block.mass', env['block']['mass']),
                         ('physics.max_step_size', env['physics']['max_step_size']),
                         ('physics.real_time_factor', env['physics']['real_time_factor'])]:
        if not math.isfinite(number) or number <= 0:
            raise ValueError(f'{name} must be finite and positive')
    for name, number in [('floor_z', env['floor_z']),
                         ('block.friction', env['block']['friction']),
                         ('task.lift_distance', env['task']['lift_distance']),
                         ('task.hold_seconds', env['task']['hold_seconds']),
                         ('task.grasp_yaw_offset', env['task']['grasp_yaw_offset'])]:
        if not math.isfinite(number):
            raise ValueError(f'{name} must be finite')
        if name not in ('floor_z', 'task.grasp_yaw_offset') and number < 0:
            raise ValueError(f'{name} must be nonnegative')
    vector(env['physics']['gravity'], 3, 'physics.gravity')
    return env


def environment_parameters(env):
    validate_environment(env)
    table, block = env['table'], env['block']
    return {
        'reference_frame': env['reference_frame'],
        'table_size': table['size'], 'table_position': table['position'],
        'table_quaternion': quaternion_from_rpy(table['orientation_rpy']),
        'block_size': block['size'],
        'block_quaternion': quaternion_from_rpy(block['orientation_rpy']),
        **dict(zip(('block_x', 'block_y', 'block_z'), block['position'])),
        'grasp_yaw': block['orientation_rpy'][2] + env['task']['grasp_yaw_offset'],
        'floor_z': float(env['floor_z']),
        'lift_distance': float(env['task']['lift_distance']),
        'hold_seconds': float(env['task']['hold_seconds']),
    }


def load_configuration(robot='', description_package='', arm_config='',
                       gripper_config='', environment_config=''):
    if not description_package:
        if not robot and not arm_config:
            raise ValueError('Supply robot, description_package, or arm_config')
        if robot:
            description_package = f'{robot}_2f85_description'
    directory = Path(share(description_package)) if description_package else None
    if not arm_config and directory is None:
        raise ValueError('arm_config is required')
    arm_path = Path(arm_config) if arm_config else directory / 'config/arm.yaml'
    # An arm file can be used outside the package with its gripper file alongside.
    gripper_path = Path(gripper_config) if gripper_config else arm_path.with_name('gripper.yaml')
    env_path = Path(environment_config) if environment_config else Path(share('robot_pick_lift')) / 'config/environment.yaml'
    arm, gripper = read_yaml(arm_path), read_yaml(gripper_path)
    env = validate_environment(read_yaml(env_path))
    for name, pose in [('arm.pose', arm['pose']), ('gripper.mount', gripper['mount'])]:
        vector(pose['position'], 3, name + '.position')
        vector(pose['orientation_rpy'], 3, name + '.orientation_rpy')
    mounting = quaternion_from_rpy(gripper['mount']['orientation_rpy'])
    grasp_offset = vector(gripper['grasp']['position'], 3, 'gripper.grasp.position')
    grasp_rotation = normalized(gripper['grasp']['quaternion'])
    rotated = multiply(multiply(mounting, grasp_offset + [0.0]),
                       [-mounting[0], -mounting[1], -mounting[2], mounting[3]])[:3]
    parameters = {**arm['task'], **gripper['task'], **environment_parameters(env),
                  'tool_to_grasp_xyz': [a+b for a, b in zip(gripper['mount']['position'], rotated)],
                  'tool_to_grasp_quaternion': multiply(mounting, grasp_rotation)}
    return arm, gripper, env, parameters
