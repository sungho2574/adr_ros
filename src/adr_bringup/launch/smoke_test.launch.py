"""Run the Jetson camera, GateNet, HUD and RTSP stream for a visual smoke test."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from adr_bringup.real_vision import vision_nodes
from adr_bringup.recording import recorder, recording_arguments
from adr_bringup.streaming import streaming_actions


def setup(context):
    path = Path(LaunchConfiguration('config').perform(context)).expanduser()
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    config['perception']['record'] = False
    stream_url = LaunchConfiguration('stream_url').perform(context).strip()
    processes = vision_nodes(config) + streaming_actions(stream_url, required=True)
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='A smoke-test process exited'))],
        ))
        for process in processes
    ]
    domain = str(config.get('ros_domain_id', 0))
    return [
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain), *handlers, *processes,
        recorder('smoke', config_yaml=yaml.safe_dump(config, sort_keys=False), trigger='launch'),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('adr_bringup')) / 'config/real.yaml'
    return LaunchDescription([
        *recording_arguments(arm_trigger=False),
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('stream_url', default_value='local',
                              description='local starts MediaMTX, or supply an RTSP publish URL'),
        OpaqueFunction(function=setup),
    ])
