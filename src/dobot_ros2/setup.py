from glob import glob
from setuptools import setup


package_name = "dobot_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=["dobot_api"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        (f"share/{package_name}/files", glob("files/*.json")),
        (f"share/{package_name}/web", glob("web/*")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="ROS2 driver wrapper for the Dobot TCP/IP Python SDK.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dobot_motion_server = dobot_ros2.driver_node:main",
            "dobot_control_console = dobot_ros2.control_console:main",
        ],
    },
)
