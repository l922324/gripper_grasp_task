"""Selectable FR3/UR3e scene with shared randomization and RGB-D cameras."""
import os
import tempfile
from pathlib import Path
import yaml
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            LogInfo, TimerAction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from fr3_vision_pick_lift.scene import generate_world, camera_pose, CAMERA_FROM_OPTICAL


def setup(context):
    robot_arm = LaunchConfiguration('robot_arm').perform(context)
    if robot_arm not in ('fr3', 'ur3e'):
        raise ValueError('robot_arm must be fr3 or ur3e')
    description_package = robot_arm + '_2f85_description'
    config = LaunchConfiguration('config').perform(context)
    with open(config) as stream:
        scene = yaml.safe_load(stream)['scene']
    seed = int(LaunchConfiguration('seed').perform(context))
    directory = Path(tempfile.mkdtemp(prefix='fr3_vision_world_'))
    world = directory / 'random_table.sdf'
    truth = generate_world(os.path.join(share(description_package), 'worlds/mvp3a_table_cube.sdf'),
                           scene, world, seed)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    args = f'{world} -r' + (' -s' if headless else '')
    egl = LaunchConfiguration('egl').perform(context).lower() == 'true'
    if egl:
        args += ' --headless-rendering'
    if robot_arm == 'fr3':
        simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share(description_package), 'launch/fr3_sim.launch.py')),
            launch_arguments={'gz_args': args}.items())
    else:
        # The upstream UR launcher owns Gazebo, robot state and arm controllers.
        # Its world_file is forwarded into Gazebo's command-line argument string.
        simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share('ur_simulation_gz'), 'launch/ur_sim_control.launch.py')),
            launch_arguments={
                'ur_type': 'ur3e', 'description_package': description_package,
                'description_file': 'ur3e_2f85.urdf.xacro',
                'runtime_config_package': description_package,
                'controllers_file': 'ur3e_2f85_controllers.yaml',
                'world_file': str(world) + (' --headless-rendering' if egl else ''),
                'gazebo_gui': 'false' if headless else 'true',
                'launch_rviz': 'false',
            }.items())
    actions = [LogInfo(msg=f'Robot arm={robot_arm}; randomized scene seed={seed}, '
                           f'block xyz/rpy={truth}; world={world}'),
               simulation,
               IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                   share(description_package), 'launch/moveit.launch.py')),
                   condition=IfCondition(LaunchConfiguration('start_moveit')))]
    if robot_arm == 'ur3e':
        # Gazebo resolves package mesh URIs through the parent share directories.
        resource_paths = [os.path.dirname(share(package)) for package in
                          ('ur_description', 'robotiq_description', description_package)]
        actions = [SetEnvironmentVariable(
            name, ':'.join(resource_paths + [os.environ.get(name, '')]))
            for name in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH')] + actions
        # The upstream launcher starts the arm controller, but not our gripper.
        actions.append(TimerAction(period=8.0, actions=[Node(
            package='controller_manager', executable='spawner',
            name='vision_robotiq_controller_spawner', output='screen',
            arguments=['robotiq_gripper_controller', '-c', '/controller_manager',
                       '--controller-manager-timeout', '60'])]))
    for camera in scene['cameras']:
        name = camera['name']
        position, _, rotation = camera_pose(camera)
        # Fortress emits point XYZ in sensor axes (+X forward), while image
        # projection uses optical axes (+Z forward). Give each a truthful frame.
        for suffix, basis in [('camera_frame', rotation @ CAMERA_FROM_OPTICAL.T),
                              ('optical_frame', rotation)]:
            q = Rotation.from_matrix(basis).as_quat()
            actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                                name=name + '_' + suffix + '_extrinsics', arguments=[
                                    '--x', str(position[0]), '--y', str(position[1]), '--z', str(position[2]),
                                    '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
                                    '--frame-id', 'world', '--child-frame-id', name + '_' + suffix]))
        images = [f'/{name}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']
        if LaunchConfiguration('bridge_images').perform(context).lower() == 'true':
            images.extend([f'/{name}/image@sensor_msgs/msg/Image[gz.msgs.Image',
                           f'/{name}/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'])
        actions.append(TimerAction(period=8.0, actions=[
            Node(package='ros_gz_bridge', executable='parameter_bridge', name=name + '_cloud_bridge',
                 arguments=[f'/{name}/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
                 output='screen', parameters=[{'use_sim_time': True,
                                              'override_frame_id': name + '_camera_frame'}]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name=name + '_image_bridge',
                 arguments=images, output='screen', parameters=[{'use_sim_time': True}])]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_arm', default_value='fr3', choices=['fr3', 'ur3e'],
                              description='Robot arm to spawn and configure for MoveIt'),
        DeclareLaunchArgument('config', default_value=os.path.join(
            share('fr3_vision_pick_lift'), 'config/vision.yaml')),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('egl', default_value='false'),
        DeclareLaunchArgument('start_moveit', default_value='true'),
        DeclareLaunchArgument('bridge_images', default_value='false'),
        OpaqueFunction(function=setup)])
