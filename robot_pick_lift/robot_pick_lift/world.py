"""Generate a self-contained SDF world from the environment configuration."""
import xml.etree.ElementTree as ET

from .configuration import validate_environment


def put(parent, tag, value=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if value is not None:
        element.text = ' '.join(str(v) for v in value) if isinstance(value, (list, tuple)) else str(value)
    return element


def world_sdf(environment):
    env = validate_environment(environment)
    sdf = ET.Element('sdf', version='1.7')
    world = put(sdf, 'world', name=env['world_name'])
    put(world, 'gravity', env['physics']['gravity'])
    physics = put(world, 'physics', name='pick_lift_physics', type='ignored')
    for key in ('max_step_size', 'real_time_factor'):
        put(physics, key, env['physics'][key])
    light = put(world, 'light', name='sun', type='directional')
    put(light, 'pose', [0, 0, 10, 0, 0, 0])
    put(light, 'diffuse', [0.8, 0.8, 0.8, 1])
    put(light, 'specular', [0.2, 0.2, 0.2, 1])
    put(light, 'direction', [-0.5, 0.1, -0.9])
    ground = put(world, 'model', name='ground_plane')
    put(ground, 'static', 'true')
    put(ground, 'pose', [0, 0, env['floor_z'], 0, 0, 0])
    link = put(ground, 'link', name='ground_link')
    for kind in ('collision', 'visual'):
        plane = put(put(put(link, kind, name='ground_' + kind), 'geometry'), 'plane')
        put(plane, 'normal', [0, 0, 1])
        put(plane, 'size', [10, 10])
    for key, model_name in [('table', 'table'), ('block', 'green_block')]:
        item = env[key]
        model = put(world, 'model', name=model_name)
        put(model, 'static', 'true' if key == 'table' else 'false')
        put(model, 'pose', item['position'] + item['orientation_rpy'])
        link = put(model, 'link', name=key + '_link')
        if key == 'block':
            mass = item['mass']
            x, y, z = item['size']
            inertial = put(link, 'inertial')
            put(inertial, 'mass', mass)
            inertia = put(inertial, 'inertia')
            for axis, value in [('ixx', mass*(y*y+z*z)/12),
                                ('iyy', mass*(x*x+z*z)/12), ('izz', mass*(x*x+y*y)/12),
                                ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
                put(inertia, axis, value)
        for kind in ('collision', 'visual'):
            element = put(link, kind, name=key + '_' + kind)
            put(put(put(element, 'geometry'), 'box'), 'size', item['size'])
            if kind == 'visual':
                material = put(element, 'material')
                put(material, 'ambient', item['color'])
                put(material, 'diffuse', item['color'])
            elif key == 'block':
                ode = put(put(put(element, 'surface'), 'friction'), 'ode')
                put(ode, 'mu', item['friction'])
                put(ode, 'mu2', item['friction'])
    return ET.tostring(sdf, encoding='unicode')
