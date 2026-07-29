import argparse
import json
import os
import tempfile
from pathlib import Path


STATE_NAMES = [
    "joint_1_pos_rad",
    "joint_2_pos_rad",
    "joint_3_pos_rad",
    "joint_4_pos_rad",
    "joint_5_pos_rad",
    "joint_6_pos_rad",
    "tcp_x_mm",
    "tcp_y_mm",
    "tcp_z_mm",
    "tcp_rx_deg",
    "tcp_ry_deg",
    "tcp_rz_deg",
    "gripper_opening_mm",
]
ACTION_NAMES = [
    "jog_x_normalized",
    "jog_y_normalized",
    "jog_z_normalized",
    "jog_rx_normalized",
    "jog_ry_normalized",
    "jog_rz_normalized",
    "gripper_target_normalized",
]


def _load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _load_steps(path: Path):
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def _lerobot_api():
    os.environ.setdefault(
        "HF_HOME", str(Path(tempfile.gettempdir()) / "dobot_lerobot_hf_cache")
    )
    import lerobot
    from lerobot.datasets import LeRobotDataset
    from lerobot.datasets.dataset_metadata import CODEBASE_VERSION

    if not str(CODEBASE_VERSION).startswith("v3."):
        raise RuntimeError(
            f"LeRobot Dataset v3 is required, found {CODEBASE_VERSION}"
        )
    return lerobot.__version__, CODEBASE_VERSION, LeRobotDataset


def check_environment():
    version, codebase_version, _ = _lerobot_api()
    return {
        "available": True,
        "lerobot_version": version,
        "codebase_version": codebase_version,
    }


def _read_rgb(path: Path):
    import cv2

    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot read image: {path}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def _features(wrist_image, global_image):
    def video_feature(image):
        height, width, channels = image.shape
        return {
            "dtype": "video",
            "shape": (height, width, channels),
            "names": ["height", "width", "channels"],
        }

    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(STATE_NAMES),),
            "names": STATE_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": (len(ACTION_NAMES),),
            "names": ACTION_NAMES,
        },
        "observation.images.wrist": video_feature(wrist_image),
        "observation.images.global": video_feature(global_image),
    }


def _vector(step: dict, key: str, size: int):
    value = step
    for part in key.split("."):
        value = value[part]
    if len(value) != size:
        raise ValueError(f"{key} must contain {size} values")
    return [float(item) for item in value]


def export_episode(raw_episode, dataset_root, repo_id, fps):
    import numpy as np

    _, codebase_version, dataset_class = _lerobot_api()
    raw_episode = Path(raw_episode).resolve()
    dataset_root = Path(dataset_root).resolve()
    metadata = _load_json(raw_episode / "metadata.json")
    steps = _load_steps(raw_episode / "steps.jsonl")
    if not metadata.get("acquisition_complete", metadata.get("complete")):
        raise ValueError("raw episode is incomplete")
    if not steps:
        raise ValueError("raw episode has no samples")
    if int(metadata.get("missed_sample_slots", 0)):
        raise ValueError("raw episode contains missed fixed-rate sample slots")
    recorded_fps = float(metadata.get("sample_rate_hz", 0.0))
    if recorded_fps and abs(recorded_fps - float(fps)) > 1e-6:
        raise ValueError(
            f"raw episode fps={recorded_fps}, requested export fps={fps}"
        )
    if recorded_fps:
        period = 1.0 / recorded_fps
        for expected_slot, step in enumerate(steps):
            if int(step.get("frame_slot", -1)) != expected_slot:
                raise ValueError(
                    f"sample {expected_slot + 1} is not on fixed-rate frame slot"
                )
            if abs(float(step.get("t", -1.0)) - expected_slot * period) > 1e-6:
                raise ValueError(
                    f"sample {expected_slot + 1} has an invalid fixed-rate timestamp"
                )

    first = steps[0]
    first_wrist = _read_rgb(raw_episode / first["images"]["wrist"]["path"])
    first_global = _read_rgb(raw_episode / first["images"]["global"]["path"])
    dataset_exists = (dataset_root / "meta" / "info.json").is_file()
    if dataset_exists:
        dataset = dataset_class.resume(repo_id=repo_id, root=dataset_root)
        if int(dataset.fps) != int(fps):
            raise ValueError(
                f"existing LeRobot dataset fps={dataset.fps}, requested fps={fps}"
            )
    else:
        if dataset_root.is_dir():
            if any(dataset_root.iterdir()):
                raise ValueError(
                    f"dataset root is not empty and is not LeRobot v3: {dataset_root}"
                )
            dataset_root.rmdir()
        dataset = dataset_class.create(
            repo_id=repo_id,
            root=dataset_root,
            fps=int(fps),
            robot_type="dobot_nova2_joystick",
            features=_features(first_wrist, first_global),
            use_videos=True,
            image_writer_threads=2,
        )

    task = str(metadata.get("task_instruction", "")).strip()
    try:
        for index, step in enumerate(steps):
            wrist = (
                first_wrist
                if index == 0
                else _read_rgb(raw_episode / step["images"]["wrist"]["path"])
            )
            global_image = (
                first_global
                if index == 0
                else _read_rgb(raw_episode / step["images"]["global"]["path"])
            )
            joints = _vector(step, "joints.position_rad", 6)
            tcp = _vector(step, "tcp_pose_mm_deg", 6)
            opening = float(step["gripper"]["opening_mm"])
            jog = _vector(step, "action.cartesian_jog_normalized", 6)
            gripper_target = float(step["action"]["gripper_target_normalized"])
            dataset.add_frame(
                {
                    "observation.state": np.asarray(
                        joints + tcp + [opening], dtype=np.float32
                    ),
                    "action": np.asarray(
                        jog + [gripper_target], dtype=np.float32
                    ),
                    "observation.images.wrist": wrist,
                    "observation.images.global": global_image,
                    "task": task,
                }
            )
        episode_index = int(dataset.num_episodes)
        dataset.save_episode()
    finally:
        dataset.finalize()

    return {
        "exported": True,
        "dataset_root": str(dataset_root),
        "repo_id": repo_id,
        "codebase_version": codebase_version,
        "episode_index": episode_index,
        "frames": len(steps),
        "task": task,
    }


def validate_dataset(dataset_root, repo_id):
    _, codebase_version, dataset_class = _lerobot_api()
    dataset = dataset_class(repo_id=repo_id, root=Path(dataset_root).resolve())
    result = {
        "valid": True,
        "dataset_root": str(Path(dataset_root).resolve()),
        "repo_id": repo_id,
        "codebase_version": codebase_version,
        "episodes": int(dataset.num_episodes),
        "frames": int(dataset.num_frames),
        "features": sorted(dataset.features),
    }
    if dataset.num_frames:
        sample = dataset[0]
        result["first_sample_keys"] = sorted(sample)
        result["first_sample_shapes"] = {
            key: list(sample[key].shape)
            for key in (
                "observation.state",
                "action",
                "observation.images.wrist",
                "observation.images.global",
            )
        }
    return result


def main(args=None):
    parser = argparse.ArgumentParser(description="Export Dobot data to LeRobot v3")
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("check")

    export_parser = subparsers.add_parser("export")
    export_parser.add_argument("raw_episode")
    export_parser.add_argument("dataset_root")
    export_parser.add_argument("repo_id")
    export_parser.add_argument("fps", type=float)

    validate_parser = subparsers.add_parser("validate")
    validate_parser.add_argument("dataset_root")
    validate_parser.add_argument("repo_id")
    parsed = parser.parse_args(args)

    if parsed.command == "check":
        result = check_environment()
    elif parsed.command == "export":
        result = export_episode(
            parsed.raw_episode,
            parsed.dataset_root,
            parsed.repo_id,
            parsed.fps,
        )
    else:
        result = validate_dataset(parsed.dataset_root, parsed.repo_id)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
