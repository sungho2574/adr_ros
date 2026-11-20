"""Jetson vision pipeline shared by the real and camera-test launches."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def enable_auto_exposure(config):
    config['camera'].update(exposure_time_range='', gain_range='',
                            isp_digital_gain_range='', ae_lock=False)


def vision_nodes(config):
    perception = config['perception']
    configs = Path(get_package_share_directory('adr_perception')) / 'configs'
    calibration = configs / Path(perception['calib_file']).expanduser()
    camera = dict(config['camera'], camera_info_url=str(calibration))
    perception = dict(perception, image_topic=camera.get('image_topic', '/adr/camera/image_raw'),
                      publish_mask_full=True, publish_rectified=True)
    return [
        Node(package='adr_camera', executable='csi_camera', output='screen', parameters=[camera]),
        Node(package='adr_perception', executable='gatenet_node', output='screen',
             parameters=[perception]),
        Node(package='adr_viz', executable='monitor', output='screen', parameters=[config['viz']]),
    ]
