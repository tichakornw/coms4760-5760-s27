"""ament_python package for the parking demo."""
from glob import glob

from setuptools import find_packages, setup

package_name = 'cs476_parking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nok Wongpiromsarn',
    maintainer_email='nok@iastate.edu',
    description='COM S 4760/5760 demo: a car parallel parks, driven by messages',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sim = cs476_parking.sim_node:main',
            'planner = cs476_parking.planner_node:main',
            'viewer = cs476_parking.viewer_node:main',
        ],
    },
)
