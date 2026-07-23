import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from dobot_ros2.handeye_common import (
    matrix_to_transform_dict,
    transform_dict_to_matrix,
)


def solve_handeye_from_samples(samples):
    if len(samples) < 3:
        raise ValueError("at least 3 samples are required")

    rotations_gripper_to_base = []
    translations_gripper_to_base = []
    rotations_target_to_camera = []
    translations_target_to_camera = []

    for sample in samples:
        base_to_flange = transform_dict_to_matrix(sample["base_to_flange"])
        board_to_camera = transform_dict_to_matrix(sample["board_to_camera"])
        rotations_gripper_to_base.append(base_to_flange[:3, :3])
        translations_gripper_to_base.append(base_to_flange[:3, 3])
        rotations_target_to_camera.append(board_to_camera[:3, :3])
        translations_target_to_camera.append(board_to_camera[:3, 3])

    rotation_camera_to_gripper, translation_camera_to_gripper = cv2.calibrateHandEye(
        rotations_gripper_to_base,
        translations_gripper_to_base,
        rotations_target_to_camera,
        translations_target_to_camera,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )

    result = np.eye(4, dtype=float)
    result[:3, :3] = rotation_camera_to_gripper
    result[:3, 3] = np.asarray(translation_camera_to_gripper, dtype=float).reshape(3)
    return result


def load_samples(samples_dir):
    path = Path(samples_dir)
    samples = []
    for sample_file in sorted(path.glob("sample_*.json")):
        samples.append(json.loads(sample_file.read_text()))
    if not samples:
        raise FileNotFoundError(f"no sample_*.json files found in {path}")
    return samples


def samples_dir_from_dataset(dataset, samples_dir):
    if dataset:
        return str(Path(dataset) / "samples")
    return samples_dir


def default_result_file(dataset, result_file):
    if result_file:
        return result_file
    if dataset:
        return str(Path(dataset) / "result.yaml")
    return "handeye_result.yaml"


def save_result(result_file, matrix, sample_count, parent_frame, child_frame):
    data = matrix_to_transform_dict(matrix)
    payload = {
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "translation": data["translation"],
        "rotation_xyzw": data["rotation_xyzw"],
        "sample_count": int(sample_count),
        "method": "CALIB_HAND_EYE_TSAI",
    }
    Path(result_file).write_text(yaml.safe_dump(payload, sort_keys=False))
    return payload


def main(argv=None):
    parser = argparse.ArgumentParser(description="Solve Dobot eye-in-hand calibration")
    parser.add_argument("--dataset", default=None)
    parser.add_argument("--samples-dir", default="handeye_samples")
    parser.add_argument("--result-file", default=None)
    parser.add_argument("--parent-frame", default="Link6")
    parser.add_argument("--child-frame", default="camera_color_optical_frame")
    args, _ = parser.parse_known_args(argv)

    samples_dir = samples_dir_from_dataset(args.dataset, args.samples_dir)
    result_file = default_result_file(args.dataset, args.result_file)
    samples = load_samples(samples_dir)
    result = solve_handeye_from_samples(samples)
    payload = save_result(
        result_file,
        result,
        len(samples),
        args.parent_frame,
        args.child_frame,
    )
    print(yaml.safe_dump(payload, sort_keys=False))


if __name__ == "__main__":
    main()
