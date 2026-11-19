"""Real vehicle vision, Betaflight telemetry, HUD streaming and recording."""
import os
from pathlib import Path
from shutil import which
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler,
                            EmitEvent, SetEnvironmentVariable, ExecuteProcess, LogInfo)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from adr_bringup.recording import recorder, recording_arguments
from adr_bringup.real_vision import vision_nodes


def streaming_actions(stream_url):
    if not stream_url:
        return []
    actions = []
    if stream_url == 'local':
        try:
            config = Path(get_package_share_directory('adr_streaming')) / 'mediamtx.yml'
        except PackageNotFoundError:
            return [LogInfo(msg='HUD streaming disabled: build the adr_streaming package.')]
        binary = which('mediamtx')
        if binary is None and os.environ.get('ADR_ROOT'):
            binary = which(str(Path(os.environ['ADR_ROOT']) / '.deps/hud/mediamtx'))
        if binary is None or not config.is_file():
            return [LogInfo(msg='HUD streaming disabled: MediaMTX executable or configuration is missing.')]
        actions.append(ExecuteProcess(
            cmd=[binary, str(config)], cwd=launch_config.log_dir, name='mediamtx', output='screen'))
        stream_url = 'rtsp://127.0.0.1:8554/hud'
    actions.append(Node(
        package='adr_streaming', executable='hud_streamer', output='screen',
        parameters=[{'url': stream_url}]))
    return actions


def setup(context):
    config_path = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    config_yaml = config_path.read_text(encoding='utf-8')
    config = yaml.safe_load(config_yaml)
    nodes = vision_nodes(config)
    telemetry = []
    if LaunchConfiguration('msp').perform(context) == 'true' and 'betaflight' in config:
        telemetry = [Node(package='adr_betaflight', executable='msp_telemetry', output='screen',
                          parameters=[config['betaflight']])]
    stream_url = LaunchConfiguration('stream_url').perform(context).strip()
    bundle = LaunchConfiguration('policy_bundle').perform(context).strip()
    policy = []
    if bundle:
        policy = [Node(
            package='adr_control', executable='policy_node', output='screen',
            parameters=[{'bundle_dir': ParameterValue(bundle, value_type=str),
                         'bench_imu': LaunchConfiguration('policy_bench_imu').perform(context) == 'true'}])]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='A required process exited'))],
    )) for node in nodes]
    return [SetEnvironmentVariable('ROS_DOMAIN_ID', str(config.get('ros_domain_id', 0))),
            *handlers, *nodes, *telemetry, *policy, recorder('real', config_yaml=config_yaml),
            *streaming_actions(stream_url)]


def generate_launch_description():
    return LaunchDescription([
        *recording_arguments(),
        DeclareLaunchArgument('config', default_value=str(
            Path(get_package_share_directory('adr_bringup')) / 'config/real.yaml')),
        DeclareLaunchArgument('stream_url', default_value='local',
                              description='local starts MediaMTX, an RTSP URL uses an existing server, '
                                          'and an empty value disables streaming'),
        DeclareLaunchArgument('msp', default_value='true', choices=['true', 'false'],
                              description='Read Betaflight telemetry using the UART configured in real.yaml'),
        DeclareLaunchArgument('policy_bundle', default_value='',
                              description='Deploy bundle directory to enable policy inference'),
        DeclareLaunchArgument('policy_bench_imu', default_value='false', choices=['true', 'false'],
                              description='Use stationary IMU inputs while bench-testing the policy'),
        OpaqueFunction(function=setup),
    ])
