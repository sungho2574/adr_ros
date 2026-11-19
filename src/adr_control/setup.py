from setuptools import setup

package_name = 'adr_control'

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
    description='100 Hz DreamerV3 policy inference for autonomous drone racing',
    license='Proprietary',
    entry_points={'console_scripts': ['policy_node = adr_control.node:main']},
)
