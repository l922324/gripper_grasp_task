"""Run observation/inference/task with the shared vision configuration."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    with open(LaunchConfiguration('config').perform(context)) as stream:
        params = yaml.safe_load(stream)['vision_pick_lift']['ros__parameters']
    for name in ('graspgen_endpoint', 'output_directory', 'perception_only', 'execute'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            if name in ('perception_only', 'execute'):
                if value.lower() not in ('true', 'false'):
                    raise ValueError(f'{name} must be true or false')
                value = value.lower() == 'true'
            params[name] = value
    return [Node(package='fr3_vision_pick_lift', executable='vision_pick_lift',
                 name='vision_pick_lift', output='screen', parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(
            share('fr3_vision_pick_lift'), 'config/vision.yaml')),
        DeclareLaunchArgument('graspgen_endpoint', default_value=''),
        DeclareLaunchArgument('output_directory', default_value=''),
        DeclareLaunchArgument('perception_only', default_value=''),
        DeclareLaunchArgument('execute', default_value=''),
        OpaqueFunction(function=setup)])
