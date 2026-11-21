"""Real vehicle vision, Betaflight bridge, optional policy, streaming and recording."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler,
                            EmitEvent, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from adr_bringup.recording import recorder, recording_arguments
from adr_bringup.real_vision import enable_auto_exposure, vision_nodes
from adr_bringup.streaming import streaming_actions


def setup(context):
    config_path = Path(LaunchConfiguration('config').perform(context)).expanduser().resolve()
    config_yaml = config_path.read_text(encoding='utf-8')
    config = yaml.safe_load(config_yaml)
    if LaunchConfiguration('auto_exposure').perform(context) == 'true':
        enable_auto_exposure(config)
        config_yaml = yaml.safe_dump(config, sort_keys=False)
    nodes = vision_nodes(config)
    msp = LaunchConfiguration('msp').perform(context) == 'true' and 'betaflight' in config
    bundle = LaunchConfiguration('policy_bundle').perform(context).strip()
    bench_imu = LaunchConfiguration('policy_bench_imu').perform(context) == 'true'
    policy_rc = LaunchConfiguration('policy_rc').perform(context) == 'true'
    if policy_rc and (not msp or bench_imu):
        raise ValueError('policy_rc requires MSP telemetry and measured IMU (policy_bench_imu:=false)')
    if bundle == 'b6p53M' or (not bundle and (policy_rc or bench_imu)):
        bundle = str(Path(get_package_share_directory('adr_control')) / 'policies/b6p53M')
    telemetry = []
    if msp:
        telemetry = [Node(package='adr_betaflight', executable='msp_telemetry', output='screen',
                          parameters=[dict(config['betaflight'], enable_rc=policy_rc)])]
    stream_url = LaunchConfiguration('stream_url').perform(context).strip()
    policy = []
    if bundle:
        policy = [Node(
            package='adr_control', executable='policy_node', output='screen',
            parameters=[{'bundle_dir': ParameterValue(bundle, value_type=str),
                         'bench_imu': bench_imu, 'enable_rc': policy_rc}])]
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
        DeclareLaunchArgument('auto_exposure', default_value='true', choices=['true', 'false'],
                              description='Use Argus auto exposure and gain'),
        DeclareLaunchArgument('stream_url', default_value='local',
                              description='local starts MediaMTX, an RTSP URL uses an existing server, '
                                          'and an empty value disables streaming'),
        DeclareLaunchArgument('msp', default_value='true', choices=['true', 'false'],
                              description='Read Betaflight telemetry using the UART configured in real.yaml'),
        DeclareLaunchArgument('policy_bundle', default_value='',
                              description='Use b6p53M or a deploy bundle directory. An empty value '
                                          'uses b6p53M when policy RC or bench IMU is enabled'),
        DeclareLaunchArgument('policy_bench_imu', default_value='false', choices=['true', 'false'],
                              description='Use stationary IMU inputs while bench-testing the policy'),
        DeclareLaunchArgument(
            'policy_rc', choices=['true', 'false'],
            default_value=AndSubstitution(
                LaunchConfiguration('msp'), NotSubstitution(LaunchConfiguration('policy_bench_imu'))),
            description='Forward policy actions through Betaflight MSP override. Enabled by '
                        'default when MSP is enabled and bench IMU is disabled'),
        OpaqueFunction(function=setup),
    ])
