import argparse
import math
from pathlib import Path

import numpy as np
import yaml

from dobot_handeye.handeye_common import (
    matrix_to_xyz_quat,
    transform_dict_to_matrix,
    xyz_quat_to_matrix,
)
from dobot_handeye.handeye_solve import default_result_file, load_samples


def load_result_matrix(result_file):
    data = yaml.safe_load(Path(result_file).read_text()) or {}
    return xyz_quat_to_matrix(data["translation"], data["rotation_xyzw"]), data


def validate_handeye_samples(samples, flange_to_camera):
    base_to_boards = []
    for sample in samples:
        base_to_flange = transform_dict_to_matrix(sample["base_to_flange"])
        board_to_camera = transform_dict_to_matrix(sample["board_to_camera"])
        base_to_boards.append(base_to_flange @ flange_to_camera @ board_to_camera)

    translations = np.asarray([matrix[:3, 3] for matrix in base_to_boards], dtype=float)
    mean_translation = translations.mean(axis=0)
    translation_errors_m = np.linalg.norm(translations - mean_translation, axis=1)

    quaternions = []
    for matrix in base_to_boards:
        _xyz, quat = matrix_to_xyz_quat(matrix)
        quat = np.asarray(quat, dtype=float)
        if quaternions and np.dot(quat, quaternions[0]) < 0:
            quat = -quat
        quaternions.append(quat)
    mean_quaternion = np.asarray(quaternions, dtype=float).mean(axis=0)
    mean_quaternion = mean_quaternion / np.linalg.norm(mean_quaternion)
    rotation_errors_deg = [
        _quat_angle_deg(quat, mean_quaternion) for quat in quaternions
    ]

    worst_index = int(np.argmax(translation_errors_m)) if len(samples) else -1
    return {
        "sample_count": len(samples),
        "board_translation_mean_m": mean_translation.tolist(),
        "translation_rms_mm": float(np.sqrt(np.mean(translation_errors_m**2)) * 1000.0),
        "translation_max_mm": float(np.max(translation_errors_m) * 1000.0),
        "rotation_rms_deg": float(
            math.sqrt(np.mean(np.asarray(rotation_errors_deg, dtype=float) ** 2))
        ),
        "rotation_max_deg": float(np.max(rotation_errors_deg)),
        "worst_sample_id": samples[worst_index].get("sample_id") if worst_index >= 0 else None,
        "per_sample": [
            {
                "sample_id": sample.get("sample_id"),
                "translation_error_mm": float(error_m * 1000.0),
                "rotation_error_deg": float(rotation_errors_deg[index]),
            }
            for index, (sample, error_m) in enumerate(zip(samples, translation_errors_m))
        ],
    }


def _quat_angle_deg(quat, reference):
    quat = np.asarray(quat, dtype=float)
    reference = np.asarray(reference, dtype=float)
    dot = abs(float(np.dot(quat, reference)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def save_validation_report(path, report):
    Path(path).write_text(yaml.safe_dump(report, sort_keys=False))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Validate Dobot hand-eye calibration")
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--result-file", default=None)
    args, _ = parser.parse_known_args(argv)

    dataset = Path(args.dataset)
    samples = load_samples(dataset / "samples")
    result_file = default_result_file(str(dataset), args.result_file)
    flange_to_camera, _result_data = load_result_matrix(result_file)
    report = validate_handeye_samples(samples, flange_to_camera)
    validation_file = dataset / "validation.yaml"
    save_validation_report(validation_file, report)
    print(yaml.safe_dump(report, sort_keys=False))
    print(f"validation_file: {validation_file}")


if __name__ == "__main__":
    main()
