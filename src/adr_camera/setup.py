from glob import glob

from setuptools import setup

setup(
    name='adr_camera',
    version='0.0.1',
    packages=['adr_camera'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/adr_camera']),
        ('share/adr_camera', ['package.xml']),
        ('share/adr_camera/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Precision Landing Maintainers',
    maintainer_email='jake@arkelectron.com',
    description='Camera inputs for autonomous drone racing',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'csi_camera = adr_camera.csi_camera:main',
            'image_relay = adr_camera.image_relay:main',
        ],
    },
)
