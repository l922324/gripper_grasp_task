"""Validate robot selection without starting Gazebo or ROS nodes."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    'vision_sim_launch', ROOT/'fr3_vision_pick_lift/launch/vision_sim.launch.py')
launch_file = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_file)


@pytest.fixture(autouse=True)
def launch_logs_in_temporary_directory(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros_logs'))


@pytest.mark.parametrize('robot,headless', [('fr3', False), ('ur3e', False), ('ur3e', True)])
def test_selected_simulation_and_moveit_share_the_camera_world(monkeypatch, tmp_path, robot, headless):
    monkeypatch.setattr(launch_file, 'share', lambda name: str(ROOT/name))
    monkeypatch.setattr(launch_file.tempfile, 'mkdtemp', lambda **kwargs: str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update(
        robot_arm=robot, config=str(ROOT/'fr3_vision_pick_lift/config/vision.yaml'),
        seed='17', headless=str(headless).lower(), egl='false',
        start_moveit='true', bridge_images='false')
    actions = launch_file.setup(context)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    simulation, moveit = includes
    sim_args = dict(simulation.launch_arguments)
    if robot == 'ur3e':
        assert sim_args['ur_type'] == 'ur3e'
        assert sim_args['description_package'] == 'ur3e_2f85_description'
        assert sim_args['runtime_config_package'] == 'ur3e_2f85_description'
        assert sim_args['gazebo_gui'] == ('false' if headless else 'true')
        assert sim_args['launch_rviz'] == 'false'
        assert Path(sim_args['world_file']) == tmp_path/'random_table.sdf'
    else:
        assert str(tmp_path/'random_table.sdf') in sim_args['gz_args']
    source = moveit.launch_description_source
    monkeypatch.setattr(source, '_get_launch_description', lambda location: LaunchDescription())
    source.get_launch_description(context)
    assert robot + '_2f85_description' in source.location
    world = ET.parse(tmp_path/'random_table.sdf').getroot().find('world')
    assert len(world.findall("model/link/sensor[@type='rgbd_camera']")) == 2
    assert world.find("model[@name='green_block']") is not None


def test_invalid_robot_rejected_before_generating_scene():
    context = LaunchContext()
    context.launch_configurations['robot_arm'] = 'invalid'
    with pytest.raises(ValueError, match='robot_arm'):
        launch_file.setup(context)
