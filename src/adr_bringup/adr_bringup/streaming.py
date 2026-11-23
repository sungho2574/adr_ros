"""Build the ROS HUD publisher and its optional local RTSP server."""
from shutil import which
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import ExecuteProcess, LogInfo
from launch.logging import launch_config
from launch_ros.actions import Node


def streaming_actions(stream_url, *, root, required=False):
    def unavailable(reason):
        if required:
            raise RuntimeError(reason)
        return [LogInfo(msg=f'HUD streaming disabled: {reason}')]

    if not stream_url:
        if required:
            raise ValueError('Set stream_url to local or an RTSP publish URL for the smoke test.')
        return []
    actions = []
    if stream_url == 'local':
        try:
            config = Path(get_package_share_directory('adr_streaming')) / 'mediamtx.yml'
        except PackageNotFoundError:
            return unavailable('Build the adr_streaming package.')
        binary = which(str(root / '.deps/hud/mediamtx')) or which('mediamtx')
        if binary is None or not config.is_file():
            return unavailable('MediaMTX executable or configuration is missing.')
        actions.append(ExecuteProcess(
            cmd=[binary, str(config)], cwd=launch_config.log_dir, name='mediamtx', output='screen'))
        stream_url = 'rtsp://127.0.0.1:8554/hud'
    actions.append(Node(
        package='adr_streaming', executable='hud_streamer', output='screen',
        parameters=[{'url': stream_url}]))
    return actions
