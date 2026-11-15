"""Create a randomized world without exporting its block pose to perception."""
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


# Columns are optical +X (right), +Y (down), +Z (forward) in camera-link axes.
CAMERA_FROM_OPTICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def camera_pose(camera):
    position = np.asarray(camera['position'], dtype=float)
    direction = np.asarray(camera['target'], dtype=float) - position
    if not np.isfinite(direction).all() or np.linalg.norm(direction) < 0.1:
        raise ValueError('Camera position and target must be finite and distinct')
    yaw = math.atan2(direction[1], direction[0])
    pitch = math.atan2(-direction[2], math.hypot(direction[0], direction[1]))
    rotation = Rotation.from_euler('xyz', [0., pitch, yaw]).as_matrix()
    return position, [0., pitch, yaw], rotation @ CAMERA_FROM_OPTICAL


def generate_world(base_world, settings, output, seed=-1):
    """Randomize yaw on the tabletop; roll/pitch stay zero for a stable cube."""
    rng = random.Random(None if seed < 0 else seed)
    size = float(settings['block_size'])
    x0, x1 = settings['x_range']
    y0, y1 = settings['y_range']
    radius = size / math.sqrt(2.)
    if (not 0.15 + radius < x0 <= x1 < 0.75 - radius or
            not -0.4 + radius < y0 <= y1 < 0.4 - radius):
        raise ValueError('Sampling bounds must keep every rotated cube corner on the table')
    pose = [rng.uniform(x0, x1), rng.uniform(y0, y1),
            float(settings['table_top']) + size / 2., 0., 0., rng.uniform(-math.pi, math.pi)]
    tree = ET.parse(base_world)
    world = tree.getroot().find('world')
    # Fortress's default detector missed off-center box/table contacts in QA.
    # Keep DART dynamics, but select its supported Bullet collision detector.
    dart = world.find('physics/dart')
    if dart is None:
        dart = ET.SubElement(world.find('physics'), 'dart')
    detector = dart.find('collision_detector')
    if detector is None:
        detector = ET.SubElement(dart, 'collision_detector')
    detector.text = 'bullet'
    world.find("model[@name='green_block']/pose").text = ' '.join(map(str, pose))
    for system, library in [('Physics', 'physics'), ('UserCommands', 'user-commands'),
                            ('SceneBroadcaster', 'scene-broadcaster'), ('Sensors', 'sensors')]:
        plugin = ET.SubElement(world, 'plugin',
                               filename='ignition-gazebo-' + library + '-system',
                               name='gz::sim::systems::' + system)
        if system == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    for camera in settings['cameras']:
        position, rpy, _ = camera_pose(camera)
        name = camera['name']
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, [*position, *rpy]))
        link = ET.SubElement(model, 'link', name='camera_link')
        visual = ET.SubElement(link, 'visual', name='body')
        ET.SubElement(visual, 'pose').text = '-0.025 0 0 0 0 0'
        ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = '0.05 0.124 0.026'
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = '0.12 0.12 0.12 1'
        ET.SubElement(material, 'diffuse').text = '0.18 0.18 0.18 1'
        sensor = ET.SubElement(link, 'sensor', name='rgbd', type='rgbd_camera')
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = str(settings['camera_rate'])
        ET.SubElement(sensor, 'topic').text = '/' + name
        optics = ET.SubElement(sensor, 'camera')
        ET.SubElement(optics, 'optical_frame_id').text = name + '_optical_frame'
        ET.SubElement(optics, 'horizontal_fov').text = str(settings['horizontal_fov'])
        image = ET.SubElement(optics, 'image')
        ET.SubElement(image, 'width').text = str(settings['camera_width'])
        ET.SubElement(image, 'height').text = str(settings['camera_height'])
        ET.SubElement(image, 'format').text = 'R8G8B8'
        clip = ET.SubElement(optics, 'clip')
        ET.SubElement(clip, 'near').text = '0.4'
        ET.SubElement(clip, 'far').text = '6.0'
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return pose  # Launch logging / offline QA only. Never supplied to the task node.
