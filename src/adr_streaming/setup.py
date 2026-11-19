from setuptools import setup

setup(
    name='adr_streaming',
    version='0.0.1',
    packages=['adr_streaming'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/adr_streaming']),
        ('share/adr_streaming', ['package.xml', 'mediamtx.yml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Precision Landing Maintainers',
    maintainer_email='jake@arkelectron.com',
    description='Low-latency live HUD streaming to MediaMTX',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['hud_streamer = adr_streaming.hud_streamer:main']},
)
