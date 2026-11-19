"""ros2 launch cs476_parking parking.launch.py: the sim, the viewer, then the planner."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="cs476_parking", executable="sim", name="sim", output="screen"),
        Node(package="cs476_parking", executable="viewer", name="viewer", output="screen"),
        Node(package="cs476_parking", executable="planner", name="planner", output="screen"),
    ])
