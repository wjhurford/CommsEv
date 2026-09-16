"""Start the world and the bridge together. Controllers are launched separately,
because a researcher starts and stops those constantly while iterating."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value=""),
        Node(package="commsev_ros", executable="world", name="commsev_world",
             output="screen", parameters=[{"scenario": scenario}]),
        Node(package="commsev_ros", executable="bridge", name="commsev_bridge",
             output="screen", parameters=[{"scenario": scenario}]),
    ])
