import os
from glob import glob
from setuptools import find_packages, setup

package_name = "aegisnav_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "console"), glob("console/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="AegisNav Maintainers",
    maintainer_email="maintainer@example.invalid",
    description="ROS 2 adapters for AegisNav",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "aegisnav-hazard-forecast = aegisnav_ros.hazard_forecast_node:main",
        "aegisnav-risk-planner = aegisnav_ros.planner_node:main",
        "aegisnav-scenario = aegisnav_ros.scenario_node:main",
        "aegisnav-visualization = aegisnav_ros.visualization_node:main",
        "aegisnav-console = aegisnav_ros.console_node:main",
        "aegisnav-director = aegisnav_ros.director_node:main",
        "aegisnav-hazard-sensor = aegisnav_ros.hazard_sensor_node:main",
    ]},
)
