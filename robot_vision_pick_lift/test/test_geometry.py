"""Configurable camera worlds and measured poses without reading spawn truth."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from robot_pick_lift.configuration import read_yaml
from robot_vision_pick_lift.configuration import validate_vision
from robot_vision_pick_lift.scene import generate_world, camera_pose
from robot_vision_pick_lift.perception import decode_cloud, crop_table, estimate_block, fuse_object_cloud

ROOT = Path(__file__).resolve().parents[1]


def configs():
    return read_yaml(ROOT/'config/environment.yaml'), read_yaml(ROOT/'config/vision.yaml')


def test_random_world_is_repeatable_and_keeps_two_sensors():
    env, vision = configs()
    first = generate_world(env, vision, 17)
    assert generate_world(env, vision, 17) == first
    assert generate_world(env, vision, 18) != first
    world = ET.fromstring(first).find('world')
    assert len(world.findall("model/link/sensor[@type='rgbd_camera']")) == 2
    assert world.find('physics/dart/collision_detector').text == 'bullet'
    assert env['block']['position'] == [0.45, 0., 0.04]  # no mutation


def test_randomization_uses_translated_rotated_table_and_block_dimensions():
    env, vision = configs()
    env['table'].update(position=[0.7, 0.2, 0.15], size=[0.8, 1.0, 0.1], orientation_rpy=[0., 0., 0.6])
    env['block']['size'] = [0.03, 0.06, 0.08]
    for seed in range(25):
        world = ET.fromstring(generate_world(env, vision, seed)).find('world')
        pose = np.array([float(v) for v in world.findtext("model[@name='green_block']/pose").split()])
        corners = np.array([[x, y, 0] for x in [-.015, .015] for y in [-.03, .03]])
        corners = Rotation.from_euler('z', pose[5]).apply(corners) + pose[:3]
        local = (corners-env['table']['position']) @ Rotation.from_euler('z', 0.6).as_matrix()
        assert np.all(np.abs(local[:, :2]) < np.array([0.4, 0.5]))
        assert pose[2] == pytest.approx(0.24)
        assert world.findtext("model[@name='green_block']/link/collision/geometry/box/size") == '0.03 0.06 0.08'


def test_fixed_pose_is_respected_when_randomization_disabled():
    env, vision = configs()
    vision['randomization']['enabled'] = False
    env['block'].update(position=[0.5, -0.1, 0.04], orientation_rpy=[0., 0., 0.3])
    world = ET.fromstring(generate_world(env, vision)).find('world')
    assert [float(v) for v in world.findtext("model[@name='green_block']/pose").split()] == [.5, -.1, .04, 0., 0., .3]


@pytest.mark.parametrize('issue', ['bounds', 'tilt', 'camera'])
def test_bad_scene_rejected(issue):
    env, vision = configs()
    if issue == 'bounds':
        env['table']['size'][0] = .1
    elif issue == 'tilt':
        env['table']['orientation_rpy'][0] = .1
    else:
        vision['cameras'][1]['name'] = vision['cameras'][0]['name']
    with pytest.raises(ValueError):
        validate_vision(env, vision)


@pytest.mark.parametrize('size,yaw', [([.04,.04,.04], -.7), ([.03,.06,.08], .42)])
def test_measured_box_pose_and_configured_crop(size, yaw):
    center = np.array([.9, -.3, .2+size[2]/2])
    points = np.array([[x,y,size[2]/2] for x in np.linspace(-size[0]/2,size[0]/2,30)
                       for y in np.linspace(-size[1]/2,size[1]/2,30)])
    points = Rotation.from_euler('z', yaw).apply(points) + center
    q = Rotation.from_euler('z', .4).as_quat()
    cropped = crop_table(points, [.9, -.3, .15], [.6,.8,.1], q, size[2])
    fused = fuse_object_cloud([cropped, cropped[::2]])
    result, detected_yaw, _ = estimate_block(fused, size, .2)
    assert result == pytest.approx(center, abs=.002)
    period = math.pi/2 if size[0] == size[1] else math.pi
    assert abs((detected_yaw-yaw+period/2) % period-period/2) < .02
    assert len(crop_table(np.array([[0., 0., .3]]), [.9,-.3,.15], [.6,.8,.1], q, size[2])) == 0


@pytest.mark.parametrize('big_endian', [False, True])
def test_padded_cloud_decode(big_endian):
    order = '>' if big_endian else '<'
    dtype = np.dtype({'names':['x','y','z','rgb'], 'formats':[order+'f4']*3+[order+'u4'],
                      'offsets':[0,4,8,16], 'itemsize':32})
    data = bytearray(160)
    rows = np.ndarray((2,2), dtype=dtype, buffer=data, strides=(80,32))
    rows['x'], rows['y'], rows['z'], rows['rgb'] = 1., 2., 3., 0x12CC34
    rows['z'][1,1] = np.nan
    fields = [SimpleNamespace(name=n, offset=o, datatype=7, count=1)
              for n,o in [('x',0),('y',4),('z',8),('rgb',16)]]
    xyz,rgb = decode_cloud(SimpleNamespace(fields=fields, data=data, width=2, height=2,
                        point_step=32, row_step=80, is_bigendian=big_endian))
    assert xyz.shape == (3,3)
    assert np.all(rgb == [0x12,0xCC,0x34])


def test_one_camera_is_not_enough():
    with pytest.raises(ValueError):
        fuse_object_cloud([np.zeros((100,3)),np.empty((0,3))])
