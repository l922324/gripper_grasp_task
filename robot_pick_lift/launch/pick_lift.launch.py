"""Run the task using the same arm, gripper and environment files as bringup."""
from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from robot_pick_lift.bringup import moveit_parameters, render_assembly
from robot_pick_lift.configuration import load_configuration


def launch_task(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    robot, package = value('robot'), value('description_package')
    if not package and robot:
        package = f'{robot}_2f85_description'
    directory = Path(get_package_share_directory(package)) if package else None
    profile = value('profile')
    actions = []
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    if profile:
        profile_path = Path(profile)
        if not profile_path.is_file():
            raise ValueError(f'Robot task profile does not exist: {profile_path}')
        parameters = [str(profile_path)]
        if value('start_moveit').lower() == 'true':
            if directory is None:
                raise ValueError('Legacy profile start_moveit requires robot or description_package')
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                str(directory / 'launch/moveit.launch.py')),
                launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()))
    else:
        arm, gripper, _, parameters = load_configuration(
            robot=robot, description_package=package, arm_config=value('arm_config'),
            gripper_config=value('gripper_config'), environment_config=value('environment_config'))
        parameters = [parameters]
        if value('start_moveit').lower() == 'true':
            generated = Path(tempfile.mkdtemp(prefix='robot_pick_lift_moveit_'))
            try:
                description, semantic, controllers = render_assembly(arm, gripper, generated)
                settings = moveit_parameters(arm, description, semantic, controllers, use_sim_time)
            except Exception:
                shutil.rmtree(generated)
                raise
            def cleanup(context):
                shutil.rmtree(generated, ignore_errors=True)
                return []
            actions.extend([RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
                            Node(package='moveit_ros_move_group', executable='move_group',
                                 output='screen', parameters=[settings])])
    overrides = {'use_sim_time': use_sim_time}
    # Empty values preserve the environment configuration instead of silently
    # replacing configured block coordinates with old launch defaults.
    for name in ('block_x', 'block_y', 'block_z', 'grasp_yaw', 'lift_distance', 'hold_seconds'):
        if value(name):
            overrides[name] = float(value(name))
    actions.append(Node(package='robot_pick_lift', executable='moveit_pick_lift',
                        name='robot_pick_lift', output='screen', parameters=[*parameters, overrides]))
    return actions


def generate_launch_description():
    arguments = {
        'robot': '', 'description_package': '', 'arm_config': '', 'gripper_config': '',
        'environment_config': '', 'profile': '', 'start_moveit': 'false', 'use_sim_time': 'true',
        'block_x': '', 'block_y': '', 'block_z': '', 'grasp_yaw': '',
        'lift_distance': '', 'hold_seconds': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()],
        OpaqueFunction(function=launch_task),
    ])
