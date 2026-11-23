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
from launch_ros.actions import Node

from adr_bringup.runtime import runtime_environment, workspace_root
from adr_bringup.real_vision import enable_auto_exposure, vision_nodes
from adr_bringup.recording import recorder, recording_arguments
from adr_bringup.streaming import streaming_actions


def setup(context, *, with_fc):
    root = workspace_root()
    path = Path(LaunchConfiguration('config').perform(context)).expanduser()
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    config['perception']['record'] = False
    if LaunchConfiguration('auto_exposure').perform(context) == 'true':
        enable_auto_exposure(config)
    stream_url = LaunchConfiguration('stream_url').perform(context).strip()
    processes = vision_nodes(config) + streaming_actions(stream_url, root=root, required=True)
    if with_fc:
        if 'betaflight' not in config:
            raise ValueError('The FC smoke test requires betaflight settings in the configuration.')
        msp_port = LaunchConfiguration('msp_port').perform(context)
        if msp_port:
            config['betaflight']['port'] = msp_port
        msp_baud = LaunchConfiguration('msp_baud').perform(context)
        if msp_baud:
            config['betaflight']['baud'] = int(msp_baud)
        processes.append(Node(package='adr_betaflight', executable='msp_telemetry',
                              output='screen', parameters=[config['betaflight']]))
    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='A smoke-test process exited'))],
        ))
        for process in processes
    ]
    domain = str(config.get('ros_domain_id', 0))
    return [
        *runtime_environment(context, root),
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain), *handlers, *processes,
        recorder('smoke', config_yaml=yaml.safe_dump(config, sort_keys=False), trigger='launch'),
    ]


def smoke_launch_description(*, with_fc):
    config = Path(get_package_share_directory('adr_bringup')) / 'config/real.yaml'
    fc_arguments = []
    if with_fc:
        fc_arguments = [
            DeclareLaunchArgument('msp_port', default_value='',
                                  description='Override betaflight.port from the config'),
            DeclareLaunchArgument('msp_baud', default_value='',
                                  description='Override betaflight.baud from the config'),
        ]
    return LaunchDescription([
        *recording_arguments(arm_trigger=False),
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('auto_exposure', default_value='true', choices=['true', 'false'],
                              description='Use Argus auto exposure and gain for the preview'),
        DeclareLaunchArgument('stream_url', default_value='local',
                              description='local starts MediaMTX, or supply an RTSP publish URL'),
        *fc_arguments,
        OpaqueFunction(function=setup, kwargs={'with_fc': with_fc}),
    ])
