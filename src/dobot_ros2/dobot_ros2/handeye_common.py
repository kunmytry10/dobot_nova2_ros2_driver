import json
import math
from pathlib import Path

import numpy as np
import yaml


class HandeyeConfigError(ValueError):
    pass


DEFAULT_HANDEYE_CONFIG = {
    "image_topic": "/camera/color/image_raw",
    "camera_info_topic": "/camera/color/camera_info",
    "base_frame": "base_link",
    "flange_frame": "Link6",
    "camera_frame": "camera_color_optical_frame",
    "samples_dir": "handeye_samples",
    "result_file": "handeye_result.yaml",
    "board": {
        "type": "charuco",
        "dictionary": "DICT_5X5_100",
        "squares_x": 12,
        "squares_y": 9,
        "square_length_m": 0.015,
        "marker_length_m": 0.01125,
        "min_charuco_corners": 12,
    },
}


def load_handeye_config(path):
    config = json.loads(json.dumps(DEFAULT_HANDEYE_CONFIG))
    if not path:
        return config

    data = yaml.safe_load(Path(path).read_text()) or {}
    params = data.get("/**", {}).get("ros__parameters", data)
    handeye = params.get("handeye", {})
    if handeye:
        _deep_update(config, handeye)
    return config


def _deep_update(target, values):
    for key, value in values.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _deep_update(target[key], value)
        else:
            target[key] = value


def node_handeye_config(node):
    config = json.loads(json.dumps(DEFAULT_HANDEYE_CONFIG))
    for name in (
        "image_topic",
        "camera_info_topic",
        "base_frame",
        "flange_frame",
        "camera_frame",
        "samples_dir",
        "result_file",
    ):
        config[name] = _declare_and_get(node, f"handeye.{name}", config[name])

    board = config["board"]
    for name in ("dictionary",):
        board[name] = _declare_and_get(node, f"handeye.board.{name}", board[name])
    for name in ("squares_x", "squares_y", "min_charuco_corners"):
        board[name] = int(_declare_and_get(node, f"handeye.board.{name}", board[name]))
    for name in ("square_length_m", "marker_length_m"):
        board[name] = float(_declare_and_get(node, f"handeye.board.{name}", board[name]))
    return config


def _declare_and_get(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def make_aruco_dictionary(name):
    import cv2

    value = getattr(cv2.aruco, name, None)
    if value is None:
        raise HandeyeConfigError(f"unsupported ArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(value)


def create_charuco_board(board_config):
    import cv2

    dictionary = make_aruco_dictionary(board_config["dictionary"])
    squares_x = int(board_config["squares_x"])
    squares_y = int(board_config["squares_y"])
    size = (squares_x, squares_y)
    square_length = float(board_config["square_length_m"])
    marker_length = float(board_config["marker_length_m"])
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(size, square_length, marker_length, dictionary)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            squares_x,
            squares_y,
            square_length,
            marker_length,
            dictionary,
        )
    raise HandeyeConfigError("OpenCV ArUco module does not support ChArUco boards")


def transform_to_matrix(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return xyz_quat_to_matrix(
        [translation.x, translation.y, translation.z],
        [rotation.x, rotation.y, rotation.z, rotation.w],
    )


def xyz_quat_to_matrix(xyz, quat):
    x, y, z, w = [float(value) for value in quat]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = [float(value) for value in xyz]
    return matrix


def matrix_to_xyz_quat(matrix):
    matrix = np.asarray(matrix, dtype=float)
    rotation = matrix[:3, :3]
    trace = np.trace(rotation)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        diagonal = np.diagonal(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / s
            x = 0.25 * s
            y = (rotation[0, 1] + rotation[1, 0]) / s
            z = (rotation[0, 2] + rotation[2, 0]) / s
        elif index == 1:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / s
            x = (rotation[0, 1] + rotation[1, 0]) / s
            y = 0.25 * s
            z = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / s
            x = (rotation[0, 2] + rotation[2, 0]) / s
            y = (rotation[1, 2] + rotation[2, 1]) / s
            z = 0.25 * s

    quat = np.array([x, y, z, w], dtype=float)
    quat = quat / np.linalg.norm(quat)
    xyz = matrix[:3, 3]
    return xyz.tolist(), quat.tolist()


def invert_matrix(matrix):
    matrix = np.asarray(matrix, dtype=float)
    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = matrix[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ matrix[:3, 3]
    return inverse


def transform_dict_to_matrix(data):
    return xyz_quat_to_matrix(data["translation"], data["rotation_xyzw"])


def matrix_to_transform_dict(matrix):
    xyz, quat = matrix_to_xyz_quat(matrix)
    return {"translation": xyz, "rotation_xyzw": quat}
