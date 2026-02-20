from glob import glob

from setuptools import setup

package_name = "aruco_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, "filters", "outputs", "viewers", "main"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="ROS 2 wrapper for ArUco SLAM using TF marker poses.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_slam_tf = aruco_slam.tf_slam_node:main",
        ],
    },
)
