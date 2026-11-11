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
    own = share('fr3_2f85_description')
    upstream = share('franka_fr3_moveit_config')
    def load(path):
        with open(path) as stream:
            return yaml.safe_load(stream)
    description = xacro.process_file(
        os.path.join(own, 'urdf/fr3_gazebo.urdf.xacro'),
        mappings={'robot_type': 'fr3', 'hand': 'false', 'gazebo': 'true',
                  'gazebo_effort': 'false'}).toxml()
    semantic = xacro.process_file(
        os.path.join(own, 'config/fr3_2f85.srdf.xacro')).toxml()
    kinematics = load(os.path.join(upstream, 'config/kinematics.yaml'))
    ompl = load(os.path.join(upstream, 'config/ompl_planning.yaml'))
    # The upstream file still uses Panda group names; bind its settings to FR3.
    ompl['fr3_arm'] = ompl.pop('panda_arm')
    ompl.pop('panda_arm_hand', None)
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
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 5.0,
        'moveit_simple_controller_manager': {
            'controller_names': ['fr3_arm_controller'],
            'fr3_arm_controller': {
                'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                'default': True, 'joints': [f'fr3_joint{i}' for i in range(1, 8)]}},
        'publish_robot_description': True, 'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='moveit_ros_move_group', executable='move_group',
             output='screen', parameters=[params])])
