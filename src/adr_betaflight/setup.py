from setuptools import setup

package_name = 'adr_betaflight'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='capstone',
    maintainer_email='capstone@example.com',
    description='Betaflight MSP telemetry over a Jetson UART',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['msp_telemetry = adr_betaflight.node:main']},
)
