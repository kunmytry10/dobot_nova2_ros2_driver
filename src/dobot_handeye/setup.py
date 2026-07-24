from setuptools import setup


package_name = "dobot_handeye"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Optional hand-eye calibration tools for Dobot ROS2 workspaces.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dobot_handeye_check = dobot_handeye.handeye_check:main",
            "dobot_handeye_capture = dobot_handeye.handeye_capture:main",
            "dobot_handeye_solve = dobot_handeye.handeye_solve:main",
            "dobot_handeye_validate = dobot_handeye.handeye_validate:main",
            "dobot_handeye_diagnose = dobot_handeye.handeye_diagnose:main",
            "dobot_handeye_tf = dobot_handeye.handeye_tf:main",
            "dobot_handeye_board_tf = dobot_handeye.handeye_board_tf:main",
        ],
    },
)
