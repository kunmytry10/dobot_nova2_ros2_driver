# Handeye Package And TF Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move hand-eye calibration tooling into an optional package while making the default Dobot bringup publish one coherent `map -> base_link -> ... -> Link6 -> camera_color_optical_frame` TF tree.

**Architecture:** Add a new Python package `dobot_handeye` that owns calibration capture/solve/validate/diagnose/TF tools. Keep Makefile command names stable and keep compatibility wrappers in `dobot_ros2` for low-risk migration. Update the URDF root from `dummy_link` to `map`, and let bringup/control-ui/rviz optionally start the handeye static TF publisher by default while leaving board TF optional.

**Tech Stack:** ROS2 Humble, ament_python, Python, OpenCV ArUco, tf2_ros, pytest, Makefile.

---

### Task 1: Add Tests For Package Split And TF Defaults

**Files:**
- Modify: `workspace/src/dobot_ros2/test/test_bringup_and_makefile.py`
- Modify: `workspace/src/dobot_ros2/test/test_handeye_tools.py`

- [ ] **Step 1: Extend tests before implementation**

Add assertions that:
- `src/dobot_handeye/package.xml` exists.
- `src/dobot_handeye/setup.py` installs `dobot_handeye_*` console scripts.
- Makefile handeye targets run `ros2 run dobot_handeye ...`.
- `dobot_bringup.launch.py` and `dobot_control_console.launch.py` include `dobot_handeye_tf` with an enable flag.
- URDF and RViz use `map`, not `dummy_link`.

- [ ] **Step 2: Run red tests**

Run: `pytest workspace/src/dobot_ros2/test -q`

Expected: FAIL because `dobot_handeye` package and launch updates are not implemented yet.

### Task 2: Create `dobot_handeye` Package

**Files:**
- Create: `workspace/src/dobot_handeye/package.xml`
- Create: `workspace/src/dobot_handeye/setup.py`
- Create: `workspace/src/dobot_handeye/setup.cfg`
- Create: `workspace/src/dobot_handeye/resource/dobot_handeye`
- Create: `workspace/src/dobot_handeye/dobot_handeye/__init__.py`
- Move/copy first: handeye modules from `workspace/src/dobot_ros2/dobot_ros2/handeye_*.py`

- [ ] **Step 1: Add package metadata**

Create a normal `ament_python` package with dependencies on `dobot_interfaces`, `rclpy`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `cv_bridge`, `python3-numpy`, `python3-opencv`, and `python3-yaml`.

- [ ] **Step 2: Move handeye implementation modules**

Move these modules into `workspace/src/dobot_handeye/dobot_handeye/` and update imports from `dobot_ros2.handeye_*` to `dobot_handeye.handeye_*`:
- `handeye_common.py`
- `handeye_capture.py`
- `handeye_check.py`
- `handeye_solve.py`
- `handeye_validate.py`
- `handeye_diagnose.py`
- `handeye_tf.py`
- `handeye_board_tf.py`

- [ ] **Step 3: Keep compatibility wrappers**

Replace old `workspace/src/dobot_ros2/dobot_ros2/handeye_*.py` files with small wrappers importing `main` and public helpers from `dobot_handeye`. This keeps older entry points from breaking during migration.

- [ ] **Step 4: Run package tests**

Run: `pytest workspace/src/dobot_ros2/test -q`

Expected: PASS after Makefile/setup assertions are updated.

### Task 3: Update Makefile And Build Targets

**Files:**
- Modify: `workspace/Makefile`

- [ ] **Step 1: Include handeye package in build**

Change build target to build up to `dobot_handeye` so both driver and calibration tools are installed.

- [ ] **Step 2: Point handeye commands to new package**

Change these targets to `ros2 run dobot_handeye ...`:
- `handeye-check`
- `handeye-capture`
- `handeye-solve`
- `handeye-validate`
- `handeye-diagnose`
- `handeye-tf`
- `handeye-board-tf`

- [ ] **Step 3: Run tests**

Run: `pytest workspace/src/dobot_ros2/test -q`

Expected: PASS.

### Task 4: Make TF Tree Coherent By Default

**Files:**
- Modify: `workspace/src/dobot_description/urdf/nova2_robot.urdf`
- Modify: `workspace/src/dobot_ros2/rviz/nova2.rviz`
- Modify: `workspace/src/dobot_ros2/launch/dobot_bringup.launch.py`
- Modify: `workspace/src/dobot_ros2/launch/dobot_control_console.launch.py`
- Modify: `workspace/src/dobot_ros2/launch/dobot_visualization.launch.py` if needed

- [ ] **Step 1: Rename root frame**

Rename URDF `dummy_link` to `map` and fixed joint `dummy_joint` parent link from `dummy_link` to `map`. Update RViz fixed/target frame to `map`.

- [ ] **Step 2: Add handeye TF launch arguments**

Add launch arguments:
- `handeye_tf`, default `true`
- `handeye_result_file`, default `$(find-pkg-share dobot_ros2)/config/handeye_result.yaml` only if a packaged default exists, otherwise use empty and conditionally start only when non-empty.

Given current result files live in datasets, prefer Makefile passing `HANDEYE_RESULT_FILE`/dataset-derived path rather than inventing a fake packaged result.

- [ ] **Step 3: Start static TF in bringup**

Add a `dobot_handeye dobot_handeye_tf` node guarded by `handeye_tf` and `handeye_result_file`.

- [ ] **Step 4: Propagate from Makefile**

Make `bringup`, `rviz`, and `control-ui` pass the same handeye TF launch args.

- [ ] **Step 5: Keep board optional**

Do not start `dobot_handeye_board_tf` from bringup; keep `make handeye-board-tf` separate.

### Task 5: Documentation And Cleanup

**Files:**
- Modify: `workspace/README.md`
- Modify: `/home/miakho/DZK/notes/dobot_nova2_handeye_calibration_record.md`
- Review: old imports/tests/package metadata

- [ ] **Step 1: Update README**

Document that handeye tools live in `dobot_handeye`; `bringup`, `rviz`, and `control-ui` can publish camera TF; `handeye_board` is optional.

- [ ] **Step 2: Update notes**

Append architecture cleanup record and new default commands.

- [ ] **Step 3: Run verification**

Run:
- `pytest workspace/src/dobot_ros2/test -q`
- `python3 -m py_compile` for `dobot_handeye` modules and compatibility wrappers.

- [ ] **Step 4: Commit**

Commit with message: `Split hand-eye tools into optional package`.
