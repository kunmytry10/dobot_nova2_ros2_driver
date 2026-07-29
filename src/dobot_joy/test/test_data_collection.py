import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from std_srvs.srv import Trigger


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dobot_joy.data_collection import DataCollectionNode  # noqa: E402
from dobot_joy.data_validate import validate_episode  # noqa: E402


def _ready_robot_state(node):
    node._remember(
        "robot",
        {
            "connected": True,
            "feedback_valid": True,
            "enable_status": 1,
            "error_status": 0,
            "robot_mode": 5,
        },
    )
    node._remember(
        "joints",
        {
            "position_rad": [0.0] * 6,
            "velocity_rad_s": [0.0] * 6,
            "effort": [0.0] * 6,
        },
    )
    node._remember("tcp_pose_mm_deg", [0.0] * 6)
    node._remember("gripper", {"opening_mm": 42.0})
    node._remember("joy", {"axes": [0.0], "buttons": [0]})
    node._remember(
        "action",
        {
            "cartesian_jog_normalized": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "gripper_target_normalized": 0.5,
        },
    )


def test_episode_writes_synchronized_wrist_and_global_images(tmp_path):
    task_file = tmp_path / "current_task.txt"
    task_file.write_text("pick up the block\n")
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"dataset_root:={tmp_path}",
            "-p",
            f"task_file:={task_file}",
            "-p",
            "lerobot_enabled:=false",
        ]
    )
    node = DataCollectionNode()
    try:
        _ready_robot_state(node)
        node._camera_info = {
            "wrist": {"width": 64, "height": 48, "k": [0.0] * 9},
            "global": {"width": 80, "height": 60, "k": [0.0] * 9},
        }
        node._refresh_task_instruction()
        node._begin_session()
        node._next_sample_monotonic -= 1.0
        wrist_image = np.zeros((48, 64, 3), dtype=np.uint8)
        wrist = node.bridge.cv2_to_imgmsg(wrist_image, encoding="bgr8")
        wrist.header.frame_id = "camera_color_optical_frame"
        wrist.header.stamp.sec = 10
        global_pixels = np.zeros((60, 80, 3), dtype=np.uint8)
        global_image = node.bridge.cv2_to_imgmsg(global_pixels, encoding="bgr8")
        global_image.header.frame_id = "global_camera_color_optical_frame"
        global_image.header.stamp.sec = 10
        global_image.header.stamp.nanosec = 20_000_000
        node._on_image_pair(wrist, global_image)
        episode_dir, complete = node._finish_session("test_complete", complete=None)

        metadata = json.loads((episode_dir / "metadata.json").read_text())
        steps = (episode_dir / "steps.jsonl").read_text().splitlines()
        step = json.loads(steps[0])

        assert complete is True
        assert metadata["format_version"] == 2
        assert metadata["complete"] is True
        assert metadata["sample_count"] == 1
        assert metadata["sample_rate_hz"] == 10.0
        assert metadata["missed_sample_slots"] == 0
        assert step["frame_slot"] == 0
        assert step["t"] == 0.0
        assert metadata["task_instruction"] == "pick up the block"
        assert abs(step["image_pair_skew_sec"] - 0.02) < 1e-9
        assert step["action"]["cartesian_jog_normalized"] == [
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        assert step["gripper"]["opening_mm"] == 42.0
        assert (episode_dir / step["images"]["wrist"]["path"]).is_file()
        assert (episode_dir / step["images"]["global"]["path"]).is_file()
        assert validate_episode(episode_dir)["valid"] is True
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_lerobot_collection_rejects_empty_task(tmp_path):
    task_file = tmp_path / "current_task.txt"
    task_file.write_text("\n")
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"dataset_root:={tmp_path}",
            "-p",
            f"task_file:={task_file}",
            "-p",
            "lerobot_enabled:=false",
        ]
    )
    node = DataCollectionNode()
    try:
        node.lerobot_enabled = True
        node._lerobot_environment = {"available": True}
        node._refresh_task_instruction()

        ready, message = node._ready_to_start()

        assert ready is False
        assert "make data-task" in message
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pending_episode_requires_accept_before_lerobot_commit(tmp_path):
    episode_dir = tmp_path / "episode_20260729_120000"
    episode_dir.mkdir()
    (episode_dir / "events.jsonl").write_text("")
    (episode_dir / "metadata.json").write_text(
        json.dumps(
            {
                "complete": True,
                "acquisition_complete": True,
                "curation_status": "pending",
                "episode_success": None,
                "lerobot": {"export": None},
            }
        )
    )
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"dataset_root:={tmp_path}",
            "-p",
            "lerobot_enabled:=false",
        ]
    )
    node = DataCollectionNode()
    try:
        node.lerobot_enabled = True
        node._export_to_lerobot = lambda path: {
            "exported": True,
            "episode_index": 3,
        }

        response = node._accept(None, Trigger.Response())

        metadata = json.loads((episode_dir / "metadata.json").read_text())
        assert response.success is True
        assert node._pending_session_dir is None
        assert metadata["curation_status"] == "accepted"
        assert metadata["episode_success"] is True
        assert metadata["lerobot"]["export"]["episode_index"] == 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_reject_keeps_pending_raw_episode_out_of_lerobot(tmp_path):
    episode_dir = tmp_path / "episode_20260729_120100"
    episode_dir.mkdir()
    (episode_dir / "events.jsonl").write_text("")
    (episode_dir / "metadata.json").write_text(
        json.dumps(
            {
                "complete": True,
                "acquisition_complete": True,
                "curation_status": "pending",
                "episode_success": None,
                "lerobot": {"export": None},
            }
        )
    )
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"dataset_root:={tmp_path}",
            "-p",
            "lerobot_enabled:=false",
        ]
    )
    node = DataCollectionNode()
    try:
        response = node._reject(None, Trigger.Response())

        metadata = json.loads((episode_dir / "metadata.json").read_text())
        assert response.success is True
        assert episode_dir.is_dir()
        assert node._pending_session_dir is None
        assert metadata["curation_status"] == "rejected"
        assert metadata["episode_success"] is False
        assert metadata["lerobot"]["export"] is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_validate_episode_rejects_missing_global_image(tmp_path):
    episode_dir = tmp_path / "episode"
    (episode_dir / "camera_info").mkdir(parents=True)
    (episode_dir / "images" / "wrist").mkdir(parents=True)
    (episode_dir / "images" / "global").mkdir(parents=True)
    metadata = {
        "format_version": 2,
        "dataset_type": "dobot_teleoperation_episode",
        "complete": True,
        "sample_count": 1,
        "configured_max_image_skew_sec": 0.05,
    }
    (episode_dir / "metadata.json").write_text(json.dumps(metadata))
    (episode_dir / "camera_info" / "wrist.json").write_text(
        json.dumps({"width": 8, "height": 6})
    )
    (episode_dir / "camera_info" / "global.json").write_text(
        json.dumps({"width": 8, "height": 6})
    )
    step = {
        "sample_id": 1,
        "images": {
            "wrist": {"path": "images/wrist/frame_000001.jpg"},
            "global": {"path": "images/global/frame_000001.jpg"},
        },
        "image_pair_skew_sec": 0.01,
        "joints": {"position_rad": [0.0] * 6},
        "tcp_pose_mm_deg": [0.0] * 6,
        "action": {"cartesian_jog_normalized": [0.0] * 6},
    }
    (episode_dir / "steps.jsonl").write_text(json.dumps(step) + "\n")

    result = validate_episode(episode_dir)

    assert result["valid"] is False
    assert any("unreadable image" in error for error in result["errors"])
