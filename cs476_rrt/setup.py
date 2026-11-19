"""ament_python package for the RRT planner (no interface generation: the path goes out
as nav_msgs/Path). The worlds (worlds/*.json) are installed to share/cs476_rrt/worlds, where
cs476_rrt.world.load_world finds them by name."""
from glob import glob

from setuptools import find_packages, setup

package_name = 'cs476_rrt'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nok Wongpiromsarn',
    maintainer_email='nok@iastate.edu',
    description='COM S 4760/5760: RRT in a ROS 2 node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'planner = cs476_rrt.planner_node:main',
        ],
    },
)
