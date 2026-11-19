"""ros2 launch adr_perception gatenet.launch.py [params_file:=/path/to/params.yaml]"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory("adr_perception"), "configs", "gatenet_params.yaml")
    params = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default,
                              description="gatenet_node parameter yaml"),
        Node(package="adr_perception", executable="gatenet_node", name="gatenet_node",
             output="screen", emulate_tty=True, parameters=[params]),
    ])
