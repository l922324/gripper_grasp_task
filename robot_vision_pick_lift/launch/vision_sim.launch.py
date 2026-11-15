"""Use shared robot bringup with a configurable randomized two-camera world."""
from scipy.spatial.transform import Rotation
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_pick_lift.simulation import create_simulation
from robot_vision_pick_lift.configuration import load_vision_configuration
from robot_vision_pick_lift.scene import generate_world, camera_pose, CAMERA_FROM_OPTICAL


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    arm, gripper, env, vision, _ = load_vision_configuration(**{
        name: value(name) for name in ('robot', 'description_package', 'arm_config',
                                      'gripper_config', 'environment_config', 'vision_config')})
    selected_seed = value('seed')
    world = generate_world(env, vision, int(selected_seed) if selected_seed else None)
    actions = create_simulation(arm, gripper, env, gui=value('gui').lower() == 'true',
                                world_xml=world, egl=value('egl').lower() == 'true')
    for camera in vision['cameras']:
        name = camera['name']
        position, _, rotation = camera_pose(camera)
        for suffix, basis in [('camera_frame', rotation @ CAMERA_FROM_OPTICAL.T),
                              ('optical_frame', rotation)]:
            quaternion = Rotation.from_matrix(basis).as_quat()
            arguments = []
            for option, number in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'),
                                       [*position, *quaternion]):
                arguments.extend(['--' + option, str(number)])
            actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                                name=name + '_' + suffix + '_extrinsics',
                                arguments=[*arguments, '--frame-id', 'world',
                                           '--child-frame-id', name + '_' + suffix],
                                parameters=[{'use_sim_time': True}]))
        images = [f'/{name}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']
        if value('bridge_images').lower() == 'true':
            images.extend([f'/{name}/image@sensor_msgs/msg/Image[gz.msgs.Image',
                           f'/{name}/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'])
        actions.extend([
            Node(package='ros_gz_bridge', executable='parameter_bridge', name=name + '_cloud_bridge',
                 arguments=[f'/{name}/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
                 output='screen', parameters=[{'use_sim_time': True,
                                              'override_frame_id': name + '_camera_frame'}]),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name=name + '_image_bridge',
                 arguments=images, output='screen', parameters=[{'use_sim_time': True}])])
    return actions


def generate_launch_description():
    arguments = {'robot': 'fr3', 'description_package': '', 'arm_config': '', 'gripper_config': '',
                 'environment_config': '', 'vision_config': '', 'seed': '', 'gui': 'true',
                 'egl': 'false', 'bridge_images': 'false'}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()],
        OpaqueFunction(function=setup)])
