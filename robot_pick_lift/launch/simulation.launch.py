"""Bring up the configured world, assembled robot, controllers, and MoveIt."""
import os
from pathlib import Path
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_pick_lift.bringup import moveit_parameters, render_assembly
from robot_pick_lift.configuration import load_configuration
from robot_pick_lift.world import world_sdf


def after_success(next_actions, label):
    def callback(event, context):
        if event.returncode != 0:
            return [LogInfo(msg=f'{label} failed (exit {event.returncode}); shutting down bringup'),
                    EmitEvent(event=Shutdown(reason=f'{label} failed'))]
        return next_actions
    return callback


def launch_simulation(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    arm, gripper, env, _ = load_configuration(**{
        name: value(name) for name in ('robot', 'description_package', 'arm_config',
                                      'gripper_config', 'environment_config')})
    directory = Path(tempfile.mkdtemp(prefix='robot_pick_lift_'))
    try:
        description, semantic, controllers = render_assembly(arm, gripper, directory)
        (directory / 'world.sdf').write_text(world_sdf(env))
        moveit = Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                      parameters=[moveit_parameters(arm, description, semantic, controllers)])
    except Exception:
        shutil.rmtree(directory)
        raise
    def cleanup(context):
        shutil.rmtree(directory, ignore_errors=True)
        return []
    resources = [str(Path(share(package)).parent) for package in arm['resource_packages']]
    environment = [SetEnvironmentVariable(name, ':'.join(resources + [os.environ.get(name, '')]))
                   for name in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH')]
    args = f'{directory / "world.sdf"} -r'
    if value('gui').lower() == 'false':
        args += ' -s'
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(share('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': args}.items())
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     output='screen', parameters=[{'robot_description': description, 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create', name='spawn_pick_robot', output='screen',
                 arguments=['-world', env['world_name'], '-file', str(directory / 'robot.urdf'),
                            '-name', arm['model'], '-allow_renaming', 'false'])
    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    spawners = [Node(package='controller_manager', executable='spawner', output='screen',
                     arguments=[name, '--controller-manager', '/controller_manager',
                                '--controller-manager-timeout', '60'])
                for name in (arm['joint_state_controller'], arm['controller'], gripper['controller'])]
    ready = [moveit, LogInfo(msg='Controllers active; starting MoveIt. Run pick_lift.launch.py with the same configuration files.')]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=spawn,
                   on_exit=after_success([spawners[0]], 'Robot spawn')))]
    for index, spawner in enumerate(spawners):
        following = [spawners[index+1]] if index+1 < len(spawners) else ready
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=spawner,
                        on_exit=after_success(following, 'Controller activation'))))
    return [*environment, RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            *handlers, gazebo, clock, publisher, spawn,
            LogInfo(msg=f'Generated simulation files: {directory}')]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value='') for name in
          ('robot', 'description_package', 'arm_config', 'gripper_config', 'environment_config')],
        DeclareLaunchArgument('gui', default_value='true', description='Run Gazebo GUI; false starts server only'),
        OpaqueFunction(function=launch_simulation),
    ])
