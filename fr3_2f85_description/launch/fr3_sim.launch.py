import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # ============================================================
    # Package paths
    # ============================================================

    package_share = get_package_share_directory(
        'fr3_2f85_description'
    )

    franka_description_share = get_package_share_directory(
        'franka_description'
    )

    ros_gz_sim_share = get_package_share_directory(
        'ros_gz_sim'
    )


    # ============================================================
    # Generate robot_description
    # ============================================================

    xacro_file = os.path.join(
        package_share,
        'urdf',
        'fr3_gazebo.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={
            'robot_type': 'fr3',
            'hand': 'false',
            'gazebo': 'true',
            'gazebo_effort': 'false',
        }
    )

    robot_description = {
        'robot_description':
            robot_description_config.toxml()
    }

    world_file = os.path.join(
        package_share,
        'worlds',
        'mvp3a_table_cube.sdf'
    )


    # ============================================================
    # Gazebo resource path
    # ============================================================

    existing_gz_path = os.environ.get(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )

    resource_paths = [os.path.dirname(franka_description_share),
                      os.path.dirname(get_package_share_directory('robotiq_description'))]
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join(resource_paths + [existing_gz_path])
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = ':'.join(
        resource_paths + [os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])

    # ============================================================
    # Gazebo
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_args')
        }.items()
    )


    # ============================================================
    # Robot State Publisher
    # ============================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description, {'use_sim_time': True}
        ]
    )


    # ============================================================
    # Spawn FR3 in Gazebo
    # ============================================================

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_fr3',
        output='screen',
        arguments=[
            '-topic',
            '/robot_description',
            '-name',
            'fr3'
        ]
    )


    # ============================================================
    # Controllers
    # ============================================================

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )


    fr3_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'fr3_arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    robotiq_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'robotiq_gripper_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )


    # Start controllers after Gazebo robot has been spawned.
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster,
                fr3_arm_controller,
                robotiq_gripper_controller
            ]
        )
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value=f'{world_file} -r'),
        gazebo,
        clock_bridge,
        robot_state_publisher,
        start_controllers,
        spawn_robot,
    ])
