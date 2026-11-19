"""Launch the Jetson CSI camera. The node defaults to IMX219 mode 3 at 30 fps."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PARAM_TYPES = {
    'sensor_id': int, 'sensor_mode': int, 'width': int, 'height': int, 'fps': float,
    'output_width': int, 'output_height': int, 'flip_method': int, 'wbmode': int,
    'interpolation_method': int, 'encoding': str, 'image_topic': str, 'camera_info_topic': str,
    'frame_id': str, 'camera_info_url': str, 'exposure_time_range': str, 'gain_range': str,
    'isp_digital_gain_range': str, 'ae_lock': str,
}


def setup(context):
    params = {}
    for name, convert in PARAM_TYPES.items():
        value = LaunchConfiguration(name).perform(context)
        if value:
            if name == 'ae_lock':
                if value not in ('true', 'false'):
                    raise ValueError('ae_lock must be true or false')
                params[name] = value == 'true'
            else:
                params[name] = convert(value)
    if 'sensor_mode' not in params and params.keys() & {'width', 'height', 'fps'}:
        params['sensor_mode'] = -1
    camera = Node(package='adr_camera', executable='csi_camera', output='screen', parameters=[params])
    return [RegisterEventHandler(OnProcessExit(
        target_action=camera,
        on_exit=[EmitEvent(event=Shutdown(reason='CSI camera exited'))])), camera]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name, default_value='') for name in PARAM_TYPES]
                             + [OpaqueFunction(function=setup)])
