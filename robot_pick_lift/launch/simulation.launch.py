"""Bring up the configured world, assembled robot, controllers, and MoveIt."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from robot_pick_lift.configuration import load_configuration
from robot_pick_lift.simulation import after_success, create_simulation


def launch_simulation(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    arm, gripper, env, _ = load_configuration(**{
        name: value(name) for name in ('robot', 'description_package', 'arm_config',
                                      'gripper_config', 'environment_config')})
    return create_simulation(arm, gripper, env, gui=value('gui').lower() == 'true')


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value='') for name in
          ('robot', 'description_package', 'arm_config', 'gripper_config', 'environment_config')],
        DeclareLaunchArgument('gui', default_value='true', description='Run Gazebo GUI; false starts server only'),
        OpaqueFunction(function=launch_simulation),
    ])
