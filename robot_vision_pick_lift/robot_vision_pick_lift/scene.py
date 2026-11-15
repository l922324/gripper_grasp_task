"""Create a randomized world without exporting its block pose to perception."""
import math
from copy import deepcopy
import random
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from robot_pick_lift.world import world_sdf
from .configuration import validate_vision



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




def generate_world(environment, settings, seed=None):
    """Return sensor SDF; random truth remains local to simulation generation."""
    validate_vision(environment, settings)
    env = deepcopy(environment)
    randomization = settings['randomization']
    if randomization['enabled']:
        chosen = randomization['seed'] if seed is None else seed
        rng = random.Random(None if chosen < 0 else chosen)
        local = [rng.uniform(*randomization['x_range']), rng.uniform(*randomization['y_range']),
                 env['table']['size'][2]/2 + env['block']['size'][2]/2]
        rotation = Rotation.from_euler('xyz', env['table']['orientation_rpy'])
        env['block']['position'] = (rotation.apply(local) + env['table']['position']).tolist()
        env['block']['orientation_rpy'] = [0., 0., env['table']['orientation_rpy'][2]
                                           + rng.uniform(*randomization['yaw_range'])]
    tree = ET.fromstring(world_sdf(env))
    world = tree.find('world')
    dart = ET.SubElement(world.find('physics'), 'dart')
    ET.SubElement(dart, 'collision_detector').text = 'bullet'
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
        ET.SubElement(clip, 'near').text = str(settings['near_clip'])
        ET.SubElement(clip, 'far').text = str(settings['far_clip'])
    return ET.tostring(tree, encoding='unicode')
