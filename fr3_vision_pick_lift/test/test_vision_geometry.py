"""Geometry contracts for random scenes, real cloud layouts and grasp rejection."""
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from fr3_vision_pick_lift.scene import generate_world, camera_pose
from fr3_vision_pick_lift.perception import decode_cloud, estimate_block, fuse_object_cloud
from fr3_vision_pick_lift.selection import select_candidates

ROOT = Path(__file__).resolve().parents[2]
CONFIG = yaml.safe_load((ROOT/'fr3_vision_pick_lift/config/vision.yaml').read_text())['scene']
BASE = ROOT/'fr3_2f85_description/worlds/mvp3a_table_cube.sdf'


def test_randomization_is_reproducible_and_changes_between_seeds(tmp_path):
    first = generate_world(BASE, CONFIG, tmp_path/'a.sdf', 1)
    assert generate_world(BASE, CONFIG, tmp_path/'b.sdf', 1) == first
    assert generate_world(BASE, CONFIG, tmp_path/'c.sdf', 2) != first
    assert first[2] == pytest.approx(0.04)
    assert first[3:5] == [0., 0.]
    world = ET.parse(tmp_path/'a.sdf').getroot().find('world')
    assert world.find('physics/dart/collision_detector').text == 'bullet'
    assert len(world.findall("model/link/sensor[@type='rgbd_camera']")) == 2
    assert BASE.read_bytes() == (ROOT/'ur3e_2f85_description/worlds/mvp3a_table_cube.sdf').read_bytes()


def test_every_sampled_cube_corner_stays_on_table(tmp_path):
    for seed in range(40):
        pose = generate_world(BASE, CONFIG, tmp_path/'random.sdf', seed)
        corners = np.array([[x, y] for x in (-0.02, 0.02) for y in (-0.02, 0.02)])
        r = Rotation.from_euler('z', pose[5]).as_matrix()[:2, :2]
        corners = corners @ r.T + pose[:2]
        assert (corners[:, 0] > 0.15).all() and (corners[:, 0] < 0.75).all()
        assert (np.abs(corners[:, 1]) < 0.4).all()


def test_both_cameras_cover_sampling_region():
    points = np.array([[x, y, z] for x in CONFIG['x_range'] for y in CONFIG['y_range']
                       for z in [0.02, 0.06]])
    tan_x = np.tan(CONFIG['horizontal_fov']/2)
    tan_y = tan_x * CONFIG['camera_height']/CONFIG['camera_width']
    for camera in CONFIG['cameras']:
        position, _, rotation = camera_pose(camera)
        optical = (points-position) @ rotation
        assert (optical[:, 2] > 0.4).all()
        assert (np.abs(optical[:, 0]/optical[:, 2]) < tan_x).all()
        assert (np.abs(optical[:, 1]/optical[:, 2]) < tan_y).all()


@pytest.mark.parametrize('big_endian', [False, True])
def test_cloud_decode_respects_row_padding_color_bits_and_endianness(big_endian):
    order = '>' if big_endian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z', 'rgb'],
                      'formats': [order+'f4']*3+[order+'u4'],
                      'offsets': [0, 4, 8, 16], 'itemsize': 32})
    data = bytearray(2*80)
    rows = np.ndarray((2, 2), dtype=dtype, buffer=data, strides=(80, 32))
    rows['x'], rows['y'], rows['z'], rows['rgb'] = 1., 2., 3., 0x12CC34
    rows['z'][1, 1] = np.nan
    fields = [SimpleNamespace(name=n, offset=o, datatype=7, count=1)
              for n, o in [('x',0),('y',4),('z',8),('rgb',16)]]
    cloud = SimpleNamespace(fields=fields, data=data, width=2, height=2,
                            point_step=32, row_step=80, is_bigendian=big_endian)
    xyz, rgb = decode_cloud(cloud)
    assert xyz.shape == (3, 3)
    assert np.all(rgb == [0x12, 0xCC, 0x34])


@pytest.mark.parametrize('yaw', [-0.7, 0.0, 0.42, 1.9])
def test_partial_cube_cloud_pose_and_square_yaw(yaw):
    xy = np.array([[x, y, 0.06] for x in np.linspace(-.02,.02,22)
                   for y in np.linspace(-.02,.02,22)])
    xy[:, :2] = xy[:, :2] @ Rotation.from_euler('z', yaw).as_matrix()[:2,:2].T + [.47,-.07]
    fused = fuse_object_cloud([xy, xy[::2]], minimum=100)
    center, detected_yaw, extents = estimate_block(fused)
    assert center == pytest.approx([.47,-.07,.04], abs=.0015)
    error = (detected_yaw-yaw+math.pi/4) % (math.pi/2)-math.pi/4
    assert abs(error) < .015
    assert extents == pytest.approx([.04,.04], abs=.002)


def test_no_single_camera_fallback():
    with pytest.raises(ValueError):
        fuse_object_cloud([np.zeros((100,3)), np.empty((0,3))])


def test_grasp_filter_keeps_score_order_and_rejects_upward_table_collision_and_bad_matrices():
    good = np.eye(4)
    good[:3,:3] = Rotation.from_euler('x', math.pi).as_matrix()
    good[:3,3] = [.45, 0., .185]
    up, low, bad = good.copy(), good.copy(), good.copy()
    up[:3,:3] = np.eye(3)
    low[2,3] = .17
    bad[0,0] = 4.
    poses = np.array([up, low, bad, good, good])
    candidates = select_candidates(poses, np.array([1., .99, .98, .8, .9]), [.45,0.,.04])
    assert [score for score, _ in candidates] == [.9, .8]
    assert np.array_equal(candidates[0][1], good)  # no snapping / centroid replacement
