import math
import sys
from pathlib import Path

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from dobot_ros2.handeye_common import (
    HandeyeConfigError,
    invert_matrix,
    load_handeye_config,
    make_aruco_dictionary,
    matrix_to_xyz_quat,
    xyz_quat_to_matrix,
)
from dobot_ros2.handeye_solve import default_result_file, solve_handeye_from_samples
from dobot_ros2.handeye_validate import validate_handeye_samples


def test_load_handeye_config_uses_meter_units_and_defaults():
    config = load_handeye_config(None)

    assert config["image_topic"] == "/camera/color/image_raw"
    assert config["camera_info_topic"] == "/camera/color/camera_info"
    assert config["base_frame"] == "base_link"
    assert config["flange_frame"] == "Link6"
    assert config["camera_frame"] == "camera_color_optical_frame"
    assert config["board"]["squares_x"] == 12
    assert config["board"]["squares_y"] == 9
    assert config["board"]["square_length_m"] == 0.015
    assert config["board"]["marker_length_m"] == 0.01125
    assert config["board"]["dictionary"] == "DICT_5X5_100"


def test_make_aruco_dictionary_rejects_unknown_name():
    with pytest.raises(HandeyeConfigError, match="unsupported ArUco dictionary"):
        make_aruco_dictionary("DICT_UNKNOWN")


def test_transform_round_trip_preserves_translation_and_rotation():
    matrix = xyz_quat_to_matrix(
        [0.12, -0.03, 0.45],
        [0.0, 0.0, math.sin(math.pi / 8.0), math.cos(math.pi / 8.0)],
    )

    xyz, quat = matrix_to_xyz_quat(matrix)
    restored = xyz_quat_to_matrix(xyz, quat)

    assert np.allclose(restored, matrix, atol=1e-9)
    assert np.allclose(invert_matrix(invert_matrix(matrix)), matrix, atol=1e-9)


def test_solve_handeye_from_synthetic_samples_recovers_transform():
    flange_to_camera = xyz_quat_to_matrix(
        [0.05, -0.02, 0.08],
        [0.0, math.sin(math.pi / 12.0), 0.0, math.cos(math.pi / 12.0)],
    )
    base_to_board = xyz_quat_to_matrix(
        [0.38, 0.06, 0.22],
        [0.0, 0.0, math.sin(math.pi / 9.0), math.cos(math.pi / 9.0)],
    )

    samples = []
    for index, angle in enumerate(np.linspace(-0.7, 0.7, 8)):
        base_to_flange = xyz_quat_to_matrix(
            [0.22 + index * 0.015, -0.16 + index * 0.02, 0.30 + index * 0.01],
            [
                math.sin(angle / 3.0),
                math.sin(angle / 5.0),
                math.sin(angle / 2.0),
                1.0,
            ],
        )
        base_to_camera = base_to_flange @ flange_to_camera
        board_to_camera = invert_matrix(base_to_camera) @ base_to_board

        base_xyz, base_quat = matrix_to_xyz_quat(base_to_flange)
        board_xyz, board_quat = matrix_to_xyz_quat(board_to_camera)
        samples.append(
            {
                "base_to_flange": {
                    "translation": base_xyz,
                    "rotation_xyzw": base_quat,
                },
                "board_to_camera": {
                    "translation": board_xyz,
                    "rotation_xyzw": board_quat,
                },
            }
        )

    result = solve_handeye_from_samples(samples)

    assert np.allclose(result, flange_to_camera, atol=1e-5)


def test_default_result_file_uses_dataset_result_yaml(tmp_path):
    dataset = tmp_path / "handeye_datasets" / "20260723_153000"
    dataset.mkdir(parents=True)

    assert default_result_file(str(dataset), None) == str(dataset / "result.yaml")
    assert default_result_file(None, None) == "handeye_result.yaml"
    assert default_result_file(str(dataset), "custom.yaml") == "custom.yaml"


def test_validate_handeye_samples_reports_fixed_board_consistency():
    flange_to_camera = xyz_quat_to_matrix(
        [0.05, -0.02, 0.08],
        [0.0, math.sin(math.pi / 12.0), 0.0, math.cos(math.pi / 12.0)],
    )
    base_to_board = xyz_quat_to_matrix(
        [0.38, 0.06, 0.22],
        [0.0, 0.0, math.sin(math.pi / 9.0), math.cos(math.pi / 9.0)],
    )

    samples = []
    for index, angle in enumerate(np.linspace(-0.6, 0.6, 6)):
        base_to_flange = xyz_quat_to_matrix(
            [0.20 + index * 0.02, -0.12 + index * 0.015, 0.32],
            [0.0, math.sin(angle / 2.0), math.sin(angle / 4.0), 1.0],
        )
        base_to_camera = base_to_flange @ flange_to_camera
        board_to_camera = invert_matrix(base_to_camera) @ base_to_board

        base_xyz, base_quat = matrix_to_xyz_quat(base_to_flange)
        board_xyz, board_quat = matrix_to_xyz_quat(board_to_camera)
        samples.append(
            {
                "sample_id": index + 1,
                "base_to_flange": {
                    "translation": base_xyz,
                    "rotation_xyzw": base_quat,
                },
                "board_to_camera": {
                    "translation": board_xyz,
                    "rotation_xyzw": board_quat,
                },
            }
        )

    report = validate_handeye_samples(samples, flange_to_camera)

    assert report["sample_count"] == 6
    assert report["translation_rms_mm"] < 1e-6
    assert report["translation_max_mm"] < 1e-6
    assert report["rotation_rms_deg"] < 1e-6
