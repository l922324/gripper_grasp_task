"""Start MoveIt against the controllers and state publisher owned by Gazebo."""
import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    own = share('ur3e_2f85_description')
    upstream = share('ur_moveit_config')
    def load(path):
        with open(path) as stream:
            return yaml.safe_load(stream)
    description = xacro.process_file(
        os.path.join(own, 'urdf/ur3e_2f85.urdf.xacro'),
        mappings={'name': 'ur', 'ur_type': 'ur3e'}).toxml()
    semantic = xacro.process_file(
        os.path.join(own, 'config/ur3e_2f85.srdf.xacro')).toxml()
    kinematics = load(os.path.join(upstream, 'config/kinematics.yaml'))
    kinematics = kinematics['/**']['ros__parameters']['robot_description_kinematics']
    ompl = load(os.path.join(upstream, 'config/ompl_planning.yaml'))
    ompl.update({
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                            'default_planner_request_adapters/FixWorkspaceBounds '
                            'default_planner_request_adapters/FixStartStateBounds '
                            'default_planner_request_adapters/FixStartStateCollision '
                            'default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1})
    params = {
        'robot_description': description,
        'robot_description_semantic': semantic,
        'robot_description_kinematics': kinematics,
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': ompl,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_manage_controllers': False,
        'moveit_simple_controller_manager': {
            'controller_names': ['joint_trajectory_controller'],
            'joint_trajectory_controller': {
                'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                'default': True, 'joints': ['shoulder_pan_joint', 'shoulder_lift_joint',
                    'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']}},
        'publish_robot_description': True, 'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='moveit_ros_move_group', executable='move_group',
             output='screen', parameters=[params])])
