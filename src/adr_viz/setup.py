from setuptools import setup

setup(
    name='adr_viz',
    version='0.0.1',
    packages=['adr_viz'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/adr_viz']),
        ('share/adr_viz', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jacob Dahl',
    maintainer_email='jake@arkelectron.com',
    description='GateNet and flight battery video HUD',
    license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'monitor = adr_viz.monitor:main',
    ]},
)
