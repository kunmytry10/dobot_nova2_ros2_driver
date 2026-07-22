# Hand-Eye Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a lightweight eye-in-hand calibration workflow for the Dobot Nova2 wrist-mounted Gemini305 camera.

**Architecture:** Keep the Orbbec/Gemini305 camera driver external. Add independent Python tools inside `dobot_ros2` that consume existing ROS topics and TF, save calibration samples, solve `Link6 -> camera_color_optical_frame`, and publish the result as a static TF. Expose the workflow through Makefile commands and README instructions consistent with the existing driver style.

**Tech Stack:** ROS2 Humble, `rclpy`, `sensor_msgs`, `tf2_ros`, OpenCV ArUco/ChArUco, YAML/JSON files, existing `dobot_ros2` ament Python package.

---

### Task 1: Add Hand-Eye Configuration

**Files:**
- Modify: `src/dobot_ros2/config/dobot_ros2.yaml`
- Test: `src/dobot_ros2/test/test_bringup_and_makefile.py`

- [ ] **Step 1: Write failing config assertions**

Add checks that the default hand-eye topics and board parameters are documented in the installed config:

```python
def test_handeye_config_documents_camera_topics_and_board():
    source = (PACKAGE_ROOT / "config" / "dobot_ros2.yaml").read_text()

    assert "handeye:" in source
    assert "/camera/color/image_raw" in source
    assert "/camera/color/camera_info" in source
    assert "camera_color_optical_frame" in source
    assert "Link6" in source
    assert "squares_x: 12" in source
    assert "squares_y: 9" in source
    assert "square_length_m: 0.015" in source
    assert "marker_length_m: 0.01125" in source
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
pytest src/dobot_ros2/test/test_bringup_and_makefile.py::test_handeye_config_documents_camera_topics_and_board -v
```

Expected: FAIL because no `handeye` config exists yet.

- [ ] **Step 3: Add config block**

Append a `handeye` block to `src/dobot_ros2/config/dobot_ros2.yaml`:

```yaml
handeye:
  image_topic: /camera/color/image_raw
  camera_info_topic: /camera/color/camera_info
  base_frame: base_link
  flange_frame: Link6
  camera_frame: camera_color_optical_frame
  samples_dir: /home/ros/ws/handeye_samples
  result_file: /home/ros/ws/handeye_result.yaml
  board:
    type: charuco
    dictionary: DICT_4X4_50
    squares_x: 12
    squares_y: 9
    square_length_m: 0.015
    marker_length_m: 0.01125
```

Use `DICT_4X4_50` as the first default and make it overridable by CLI args, because the exact board dictionary still needs physical verification.

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
pytest src/dobot_ros2/test/test_bringup_and_makefile.py::test_handeye_config_documents_camera_topics_and_board -v
```

Expected: PASS.

### Task 2: Add Topic/TF Check Tool

**Files:**
- Create: `src/dobot_ros2/dobot_ros2/handeye_check.py`
- Modify: `src/dobot_ros2/setup.py`
- Modify: `src/dobot_ros2/test/test_bringup_and_makefile.py`

- [ ] **Step 1: Write failing entrypoint assertions**

Extend the Makefile/entrypoint test:

```python
def test_handeye_console_entrypoints_are_installed():
    setup = (PACKAGE_ROOT / "setup.py").read_text()

    assert "dobot_handeye_check = dobot_ros2.handeye_check:main" in setup
    assert "dobot_handeye_capture = dobot_ros2.handeye_capture:main" in setup
    assert "dobot_handeye_solve = dobot_ros2.handeye_solve:main" in setup
    assert "dobot_handeye_tf = dobot_ros2.handeye_tf:main" in setup
```

- [ ] **Step 2: Implement `handeye_check.py`**

Create a node that checks these topics exist and prints a concise status:

```text
/camera/color/image_raw
/camera/color/camera_info
/joint_states
/tcp_pose
/dobot_state
/tf
/tf_static
```

It should exit with code `0` only when all required topics are present.

- [ ] **Step 3: Register entrypoint**

Add to `setup.py`:

```python
"dobot_handeye_check = dobot_ros2.handeye_check:main",
```

- [ ] **Step 4: Run test**

Run:

```bash
pytest src/dobot_ros2/test/test_bringup_and_makefile.py::test_handeye_console_entrypoints_are_installed -v
```

Expected: FAIL until all entrypoints are added in later tasks; keep the test and add missing entrypoints incrementally.

### Task 3: Add ChArUco Detection and Sample Capture

**Files:**
- Create: `src/dobot_ros2/dobot_ros2/handeye_common.py`
- Create: `src/dobot_ros2/dobot_ros2/handeye_capture.py`
- Modify: `src/dobot_ros2/setup.py`
- Test: `src/dobot_ros2/test/test_handeye_tools.py`

- [ ] **Step 1: Write unit tests for board config parsing**

Test that board values are loaded with meters, not millimeters, and that unsupported dictionaries raise a clear error.

- [ ] **Step 2: Implement `handeye_common.py`**

Provide helpers for:

```python
load_handeye_config(path: str | None) -> dict
create_charuco_board(config: dict)
make_aruco_dictionary(name: str)
matrix_to_xyz_quat(matrix)
xyz_quat_to_matrix(xyz, quat)
```

- [ ] **Step 3: Implement `handeye_capture.py`**

The capture node subscribes to `/camera/color/image_raw` and `/camera/color/camera_info`, listens to TF for `base_link -> Link6`, detects ChArUco, and saves one sample per Enter key press:

```json
{
  "sample_id": 1,
  "stamp_sec": 123.456,
  "base_frame": "base_link",
  "flange_frame": "Link6",
  "camera_frame": "camera_color_optical_frame",
  "base_to_flange": {"translation": [0, 0, 0], "rotation_xyzw": [0, 0, 0, 1]},
  "camera_to_board": {"translation": [0, 0, 0], "rotation_xyzw": [0, 0, 0, 1]},
  "charuco_corners": 42
}
```

Reject a sample if the board is not detected or TF is unavailable.

- [ ] **Step 4: Register entrypoint**

Add:

```python
"dobot_handeye_capture = dobot_ros2.handeye_capture:main",
```

### Task 4: Add Hand-Eye Solver

**Files:**
- Create: `src/dobot_ros2/dobot_ros2/handeye_solve.py`
- Modify: `src/dobot_ros2/setup.py`
- Test: `src/dobot_ros2/test/test_handeye_tools.py`

- [ ] **Step 1: Write synthetic solve test**

Generate synthetic paired transforms with a known `flange -> camera` transform and assert the solver recovers it within a small tolerance.

- [ ] **Step 2: Implement solver**

Read sample JSON files, convert transforms to rotation/translation arrays, call:

```python
cv2.calibrateHandEye(
    R_gripper2base,
    t_gripper2base,
    R_target2cam,
    t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI,
)
```

Save:

```yaml
parent_frame: Link6
child_frame: camera_color_optical_frame
translation: [x, y, z]
rotation_xyzw: [x, y, z, w]
sample_count: 20
method: CALIB_HAND_EYE_TSAI
```

- [ ] **Step 3: Register entrypoint**

Add:

```python
"dobot_handeye_solve = dobot_ros2.handeye_solve:main",
```

### Task 5: Add Static TF Publisher Wrapper

**Files:**
- Create: `src/dobot_ros2/dobot_ros2/handeye_tf.py`
- Modify: `src/dobot_ros2/setup.py`
- Test: `src/dobot_ros2/test/test_handeye_tools.py`

- [ ] **Step 1: Write result YAML parser test**

Assert the parser reads translation/quaternion and frame names from `handeye_result.yaml`.

- [ ] **Step 2: Implement `handeye_tf.py`**

Publish one `geometry_msgs/msg/TransformStamped` on `/tf_static` using `tf2_ros.StaticTransformBroadcaster`.

- [ ] **Step 3: Register entrypoint**

Add:

```python
"dobot_handeye_tf = dobot_ros2.handeye_tf:main",
```

### Task 6: Add Makefile Commands and README Workflow

**Files:**
- Modify: `Makefile`
- Modify: `README.md`
- Modify: `src/dobot_ros2/test/test_bringup_and_makefile.py`

- [ ] **Step 1: Add Makefile targets**

Add:

```makefile
handeye-check:
	$(ROS_ENV) && ros2 run dobot_ros2 dobot_handeye_check --ros-args --params-file $(PARAMS)

handeye-capture:
	$(ROS_ENV) && ros2 run dobot_ros2 dobot_handeye_capture --ros-args --params-file $(PARAMS)

handeye-solve:
	$(ROS_ENV) && ros2 run dobot_ros2 dobot_handeye_solve --ros-args --params-file $(PARAMS)

handeye-tf:
	$(ROS_ENV) && ros2 run dobot_ros2 dobot_handeye_tf --ros-args --params-file $(PARAMS)
```

- [ ] **Step 2: Add tests for Makefile targets**

Assert all four targets are present and use `ros2 run dobot_ros2`.

- [ ] **Step 3: Document operator workflow**

README should describe:

```text
1. Start Dobot bringup.
2. Start Orbbec Gemini305 driver separately.
3. Run make handeye-check.
4. Fix board on table.
5. Move robot to 15-30 diverse poses.
6. Run make handeye-capture and press Enter at each stable pose.
7. Run make handeye-solve.
8. Run make handeye-tf.
9. Verify TF in RViz.
```

### Task 7: Verify and Commit

**Files:**
- All modified hand-eye files

- [ ] **Step 1: Run Python syntax checks**

Run:

```bash
python3 -m py_compile src/dobot_ros2/dobot_ros2/handeye_*.py
```

Expected: no output.

- [ ] **Step 2: Run tests**

Run:

```bash
pytest src/dobot_ros2/test
```

Expected: all tests pass.

- [ ] **Step 3: Build package**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to dobot_ros2
```

Expected: `dobot_interfaces`, `dobot_description`, and `dobot_ros2` build successfully.

- [ ] **Step 4: Commit**

Run:

```bash
git add Makefile README.md src/dobot_ros2
git commit -m "Add hand-eye calibration tools"
```

