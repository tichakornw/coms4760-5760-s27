"""ros2 launch cs476_rrt rrt.launch.py world:=challenge seed:=3: the planner on a world."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="practice",
                              description="practice, challenge, or a path to a world file"),
        DeclareLaunchArgument("seed", default_value="0", description="the random seed"),
        Node(package="cs476_rrt", executable="planner", name="planner", output="screen",
             parameters=[{"world": LaunchConfiguration("world"),
                          "seed": LaunchConfiguration("seed")}]),
    ])
