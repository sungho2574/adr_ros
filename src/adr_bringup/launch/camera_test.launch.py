"""Run the Jetson camera, GateNet, and HUD using the real flight settings."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from adr_bringup.real_vision import vision_nodes


def setup(context):
    path = Path(LaunchConfiguration('config').perform(context)).expanduser()
    config = yaml.safe_load(path.read_text())
    nodes = vision_nodes(config)
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='A camera test node exited'))],
        ))
        for node in nodes
    ]
    domain = LaunchConfiguration('domain_id').perform(context) or str(config.get('ros_domain_id', 0))
    return [
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain),
        *handlers, *nodes,
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/adr/monitor/image'],
            condition=IfCondition(LaunchConfiguration('view')),
            output='screen'),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('adr_bringup')) / 'config/real.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('domain_id', default_value='',
                              description='Override the ROS domain set by the camera test config'),
        DeclareLaunchArgument('view', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
