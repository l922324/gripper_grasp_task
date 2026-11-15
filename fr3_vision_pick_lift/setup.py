from glob import glob
from setuptools import find_packages, setup

name = 'fr3_vision_pick_lift'
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                ('share/' + name, ['package.xml', 'README.md', 'requirements-client.txt']),
                ('share/' + name + '/launch', glob('launch/*.launch.py')),
                ('share/' + name + '/config', glob('config/*.yaml'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='robotspec', maintainer_email='robotspec@todo.todo',
    description='Dual RGB-D observation and GraspGenX integration for FR3 pick and lift',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'vision_pick_lift = fr3_vision_pick_lift.vision_pick_lift:main']})
