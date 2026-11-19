import glob
import os

from setuptools import find_packages, setup

package_name = "adr_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "configs"),
         [f for f in glob.glob("configs/*") if os.path.isfile(f)]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="capstone",
    maintainer_email="capstone@example.com",
    description="GateNet real-time gate segmentation node (64x64 mask for the policy)",
    license="Proprietary",
    entry_points={"console_scripts": ["gatenet_node = adr_perception.node:main"]},
)
