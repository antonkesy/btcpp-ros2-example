import os
from glob import glob

from setuptools import setup

package_name = "actions_py"

setup(
    name=package_name,
    version="0.15.1",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Anton Kesy",
    author_email="antonkesy@gmail.com",
    keywords=["ROS"],
    classifiers=[
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Examples of action servers using rclpy.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "print_action_server = " + package_name + ".print_action_server:main",
            "runtime_action_server = " + package_name + ".runtime_action_server:main",
        ],
    },
)
