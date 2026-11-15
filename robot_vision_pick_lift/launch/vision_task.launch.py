"""Detect, save the observed pose, then execute the shared pick/lift by default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_vision_pick_lift.configuration import load_vision_configuration


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    arm, _, _, _, params = load_vision_configuration(**{
        name: value(name) for name in ('robot', 'description_package', 'arm_config',
                                      'gripper_config', 'environment_config', 'vision_config')})
    params['robot_model'] = arm['model']
    for name in ('output_directory', 'perception_only', 'execute'):
        selected = value(name)
        if selected:
            if name != 'output_directory':
                if selected.lower() not in ('true', 'false'):
                    raise ValueError(f'{name} must be true or false')
                selected = selected.lower() == 'true'
            params[name] = selected
    return [Node(package='robot_vision_pick_lift', executable='vision_pick_lift',
                 name='robot_vision_pick_lift', output='screen', parameters=[params])]


def generate_launch_description():
    arguments = {'robot': 'fr3', 'description_package': '', 'arm_config': '',
                 'gripper_config': '', 'environment_config': '', 'vision_config': '',
                 'output_directory': '', 'perception_only': '', 'execute': ''}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()],
        OpaqueFunction(function=setup)])
