from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


def test_camera_package_wraps_both_official_drivers_and_viewers():
    package_xml = (PACKAGE_ROOT / "package.xml").read_text()
    global_launch = (
        PACKAGE_ROOT / "launch" / "realsense_global.launch.py"
    ).read_text()
    dual_launch = (PACKAGE_ROOT / "launch" / "dual_camera.launch.py").read_text()
    view_launch = (PACKAGE_ROOT / "launch" / "camera_view.launch.py").read_text()

    assert "<exec_depend>orbbec_camera</exec_depend>" in package_xml
    assert "<exec_depend>realsense2_camera</exec_depend>" in package_xml
    assert "<exec_depend>rqt_image_view</exec_depend>" in package_xml
    assert 'FindPackageShare("realsense2_camera")' in global_launch
    assert 'default_value="global_camera"' in global_launch
    assert "gemini305.launch.py" in dual_launch
    assert "realsense_global.launch.py" in dual_launch
    assert "rqt_image_view" in view_launch
    assert "/camera/color/image_raw" in view_launch
    assert "/global_camera/color/image_raw" in view_launch


def test_makefile_keeps_camera_drivers_and_viewers_separate():
    source = (WORKSPACE_ROOT / "Makefile").read_text()

    assert "camera-wrist:" in source
    assert "camera-global:" in source
    assert "camera-view:" in source
    assert "ros2 launch dobot_camera dual_camera.launch.py" in source
    assert "ros2 launch dobot_camera camera_view.launch.py" in source
    camera_view_recipe = source.split("camera-view:", 1)[1].split("camera-topics:", 1)[0]
    assert "dual_camera.launch.py" not in camera_view_recipe
