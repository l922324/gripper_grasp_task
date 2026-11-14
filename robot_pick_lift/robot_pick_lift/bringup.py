"""Robot-independent Gazebo assembly and MoveIt configuration."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml

from .configuration import read_yaml, resource


def controller_configuration(arm, gripper):
    sources = [read_yaml(resource(arm['controller_file'])),
               read_yaml(resource(gripper['controller_file']))]
    manager = {'update_rate': sources[0]['controller_manager']['ros__parameters']['update_rate'],
               'use_sim_time': True}
    result = {'controller_manager': {'ros__parameters': manager}}
    for name, source in [(arm['joint_state_controller'], sources[0]),
                         (arm['controller'], sources[0]), (gripper['controller'], sources[1])]:
        manager[name] = copy.deepcopy(source['controller_manager']['ros__parameters'][name])
        if name in source:
            result[name] = copy.deepcopy(source[name])
    return result


def render_assembly(arm, gripper, output_directory):
    directory = Path(output_directory)
    directory.mkdir(parents=True, exist_ok=True)
    controllers = controller_configuration(arm, gripper)
    controller_path = directory / 'controllers.yaml'
    controller_path.write_text(yaml.safe_dump(controllers, sort_keys=False))
    mappings = {key: str(value) for key, value in arm['xacro_arguments'].items()}
    def text(values):
        return ' '.join(str(v) for v in values)
    mappings.update(xyz=text(arm['pose']['position']), rpy=text(arm['pose']['orientation_rpy']),
                    gripper_xyz=text(gripper['mount']['position']),
                    gripper_rpy=text(gripper['mount']['orientation_rpy']),
                    simulation_controllers=str(controller_path))
    description = xacro.process_file(str(resource(arm['description'])), mappings=mappings).toxml()
    semantic = xacro.process_file(str(resource(arm['semantic']))).toxml()
    model = ET.fromstring(description)
    links = {link.get('name') for link in model.findall('link')}
    if arm['task']['tool_link'] not in links:
        raise ValueError('Configured planning tool link is absent from the robot description')
    for link in arm['task']['floor_contact_links']:
        if link not in links:
            raise ValueError(f'Floor-contact link is absent from description: {link}')
    group = ET.fromstring(semantic).find(f"group[@name='{arm['task']['planning_group']}']")
    if group is None:
        raise ValueError('Configured planning group is absent from SRDF')
    joints = controllers[arm['controller']]['ros__parameters']['joints']
    known_joints = {joint.get('name') for joint in model.findall('joint')}
    if not set(joints).issubset(known_joints):
        raise ValueError('Arm controller contains joints absent from description')
    if gripper['task']['gripper_joint'] not in known_joints:
        raise ValueError('Configured gripper joint is absent from description')
    (directory / 'robot.urdf').write_text(description)
    return description, semantic, controllers


def moveit_parameters(arm, description, semantic, controllers, use_sim_time=True):
    kinematics = read_yaml(resource(arm['kinematics']))
    for key in arm.get('kinematics_keys', []):
        kinematics = kinematics[key]
    if arm['task']['planning_group'] not in kinematics:
        raise ValueError('Planning group is absent from kinematics configuration')
    ompl = read_yaml(resource(arm['ompl']))
    for old, new in arm.get('ompl_group_aliases', {}).items():
        ompl[new] = ompl.pop(old)
    for group in arm.get('ompl_remove_groups', []):
        ompl.pop(group, None)
    ompl.update({
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                            'default_planner_request_adapters/FixWorkspaceBounds '
                            'default_planner_request_adapters/FixStartStateBounds '
                            'default_planner_request_adapters/FixStartStateCollision '
                            'default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1})
    controller = arm['controller']
    return {
        'robot_description': description, 'robot_description_semantic': semantic,
        'robot_description_kinematics': kinematics,
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 5.0,
        'moveit_simple_controller_manager': {'controller_names': [controller], controller: {
            'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
            'default': True, 'joints': controllers[controller]['ros__parameters']['joints']}},
        'publish_robot_description': True, 'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
        'use_sim_time': use_sim_time,
    }
