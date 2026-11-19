from setuptools import setup

setup(
    name='adr_logging',
    version='0.0.1',
    packages=['adr_logging'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/adr_logging']),
        ('share/adr_logging', ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='Precision Landing Maintainers',
    maintainer_email='jake@arkelectron.com',
    description='Racing telemetry and HUD rosbag recorder',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'flight_recorder = adr_logging.flight_recorder:main',
    ]},
)
