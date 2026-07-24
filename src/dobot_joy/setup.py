from glob import glob
from setuptools import setup


package_name = "dobot_joy"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Joystick teleoperation tools for Dobot ROS2 workspaces.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dobot_joy_teleop = dobot_joy.joy_teleop:main",
        ],
    },
)
