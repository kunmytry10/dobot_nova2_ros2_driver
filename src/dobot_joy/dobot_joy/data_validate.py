import argparse
import json
from pathlib import Path

import cv2


def _load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read {path}: {exc}") from exc


def _load_steps(path: Path):
    steps = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ValueError(f"cannot read {path}: {exc}") from exc
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            steps.append(json.loads(line))
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON in {path}:{line_number}: {exc}") from exc
    return steps


def _safe_image_path(episode_dir: Path, relative_path: str):
    path = (episode_dir / relative_path).resolve()
    try:
        path.relative_to(episode_dir.resolve())
    except ValueError as exc:
        raise ValueError(f"image path escapes episode: {relative_path}") from exc
    return path


def _check_vector(step, key, expected_size, sample_id, errors):
    value = step
    for part in key.split("."):
        if not isinstance(value, dict) or part not in value:
            errors.append(f"sample {sample_id}: missing {key}")
            return
        value = value[part]
    if not isinstance(value, list) or len(value) != expected_size:
        errors.append(f"sample {sample_id}: {key} must have {expected_size} values")


def _validate_v1(episode_dir, metadata, steps, errors, warnings):
    for step in steps:
        sample_id = step.get("sample_id", "?")
        relative_path = step.get("color_image")
        if not relative_path:
            errors.append(f"sample {sample_id}: missing color_image")
            continue
        image_path = _safe_image_path(episode_dir, relative_path)
        if cv2.imread(str(image_path)) is None:
            errors.append(f"sample {sample_id}: unreadable image {relative_path}")
        _check_vector(
            step,
            "action.cartesian_jog_normalized",
            6,
            sample_id,
            errors,
        )
    warnings.append("format_version=1 contains only the wrist camera")


def _validate_v2(episode_dir, metadata, steps, errors, warnings):
    camera_info = {}
    for camera in ("wrist", "global"):
        info_path = episode_dir / "camera_info" / f"{camera}.json"
        try:
            camera_info[camera] = _load_json(info_path)
        except ValueError as exc:
            errors.append(str(exc))

    configured_skew = float(metadata.get("configured_max_image_skew_sec", 0.05))
    referenced_images = {"wrist": set(), "global": set()}
    sample_rate_hz = float(metadata.get("sample_rate_hz", 0.0))
    expected_period = 1.0 / sample_rate_hz if sample_rate_hz > 0.0 else None
    for expected_id, step in enumerate(steps, start=1):
        sample_id = step.get("sample_id", "?")
        if sample_id != expected_id:
            errors.append(
                f"sample sequence gap: expected {expected_id}, found {sample_id}"
            )
        if expected_period is not None and "frame_slot" in step:
            expected_slot = expected_id - 1
            frame_slot = int(step.get("frame_slot", -1))
            if frame_slot != expected_slot:
                errors.append(
                    f"sample {sample_id}: frame_slot={frame_slot}, "
                    f"expected {expected_slot}"
                )
            expected_t = frame_slot * expected_period
            actual_t = float(step.get("t", -1.0))
            if abs(actual_t - expected_t) > 1e-6:
                errors.append(
                    f"sample {sample_id}: t={actual_t:.6f}s does not match "
                    f"fixed-rate slot {expected_t:.6f}s"
                )
        images = step.get("images", {})
        for camera in ("wrist", "global"):
            image_data = images.get(camera, {})
            relative_path = image_data.get("path")
            if not relative_path:
                errors.append(f"sample {sample_id}: missing images.{camera}.path")
                continue
            referenced_images[camera].add(relative_path)
            image_path = _safe_image_path(episode_dir, relative_path)
            image = cv2.imread(str(image_path))
            if image is None:
                errors.append(f"sample {sample_id}: unreadable image {relative_path}")
                continue
            info = camera_info.get(camera, {})
            expected_size = (int(info.get("width", 0)), int(info.get("height", 0)))
            actual_size = (int(image.shape[1]), int(image.shape[0]))
            if all(expected_size) and actual_size != expected_size:
                errors.append(
                    f"sample {sample_id}: {camera} image size {actual_size} "
                    f"does not match CameraInfo {expected_size}"
                )
        try:
            skew = float(step["image_pair_skew_sec"])
        except (KeyError, TypeError, ValueError):
            errors.append(f"sample {sample_id}: invalid image_pair_skew_sec")
        else:
            if skew > configured_skew + 1e-6:
                errors.append(
                    f"sample {sample_id}: image skew {skew:.6f}s exceeds "
                    f"{configured_skew:.6f}s"
                )
        _check_vector(step, "joints.position_rad", 6, sample_id, errors)
        _check_vector(step, "tcp_pose_mm_deg", 6, sample_id, errors)
        _check_vector(
            step,
            "action.cartesian_jog_normalized",
            6,
            sample_id,
            errors,
        )

    for camera in ("wrist", "global"):
        image_dir = episode_dir / "images" / camera
        actual_images = {
            str(path.relative_to(episode_dir)) for path in image_dir.glob("*.jpg")
        }
        orphaned = sorted(actual_images - referenced_images[camera])
        if orphaned:
            errors.append(f"{camera}: {len(orphaned)} unreferenced image(s)")

    if int(metadata.get("dropped_pairs", 0)):
        warnings.append(f"dropped_pairs={metadata['dropped_pairs']}")
    if int(metadata.get("write_errors", 0)):
        warnings.append(f"write_errors={metadata['write_errors']}")
    if int(metadata.get("missed_sample_slots", 0)):
        errors.append(f"missed_sample_slots={metadata['missed_sample_slots']}")


def validate_episode(episode_dir, allow_incomplete=False):
    episode_dir = Path(episode_dir)
    errors = []
    warnings = []
    try:
        metadata = _load_json(episode_dir / "metadata.json")
        steps = _load_steps(episode_dir / "steps.jsonl")
    except ValueError as exc:
        return {"valid": False, "errors": [str(exc)], "warnings": []}

    if metadata.get("dataset_type") != "dobot_teleoperation_episode":
        errors.append("unsupported dataset_type")
    if not metadata.get("complete") and not allow_incomplete:
        errors.append("episode is marked incomplete")
    if int(metadata.get("sample_count", -1)) != len(steps):
        errors.append(
            f"metadata sample_count={metadata.get('sample_count')} but found "
            f"{len(steps)} step(s)"
        )
    if not steps:
        errors.append("episode has no steps")

    version = int(metadata.get("format_version", 0))
    try:
        if version == 1:
            _validate_v1(episode_dir, metadata, steps, errors, warnings)
        elif version == 2:
            _validate_v2(episode_dir, metadata, steps, errors, warnings)
        else:
            errors.append(f"unsupported format_version={version}")
    except ValueError as exc:
        errors.append(str(exc))

    return {
        "valid": not errors,
        "episode": str(episode_dir),
        "format_version": version,
        "sample_count": len(steps),
        "errors": errors,
        "warnings": warnings,
    }


def main(args=None):
    parser = argparse.ArgumentParser(description="Validate a Dobot teleoperation episode")
    parser.add_argument("episode")
    parser.add_argument("--allow-incomplete", action="store_true")
    parsed = parser.parse_args(args)
    result = validate_episode(parsed.episode, parsed.allow_incomplete)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["valid"] else 1)


if __name__ == "__main__":
    main()
