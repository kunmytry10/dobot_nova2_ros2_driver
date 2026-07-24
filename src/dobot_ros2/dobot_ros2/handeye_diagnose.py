import argparse
from pathlib import Path

import yaml

from dobot_ros2.handeye_solve import (
    HANDEYE_METHODS,
    default_result_file,
    load_samples,
    solve_handeye_from_samples,
)
from dobot_ros2.handeye_validate import validate_handeye_samples


def diagnose_handeye_samples(samples, methods=None, max_leave_one_out=None):
    methods = methods or list(HANDEYE_METHODS)
    baseline = None
    method_reports = []
    for method in methods:
        try:
            transform = solve_handeye_from_samples(samples, method=method)
            report = validate_handeye_samples(samples, transform)
            method_report = _summary_from_validation(report)
            method_report["method"] = method
            method_reports.append(method_report)
            if method == "TSAI":
                baseline = dict(method_report)
        except Exception as exc:
            method_reports.append({"method": method, "success": False, "message": str(exc)})

    successful_methods = [report for report in method_reports if report.get("success", True)]
    best_method = None
    if successful_methods:
        best_method = min(successful_methods, key=_score_report)["method"]
    if baseline is None and successful_methods:
        baseline = dict(min(successful_methods, key=_score_report))

    leave_one_out = []
    if len(samples) > 3:
        for index, sample in enumerate(samples):
            subset = samples[:index] + samples[index + 1 :]
            try:
                transform = solve_handeye_from_samples(subset, method="TSAI")
                report = validate_handeye_samples(subset, transform)
                item = _summary_from_validation(report)
                item["removed_sample_id"] = sample.get("sample_id", index + 1)
                item["removed_index"] = index
                if baseline:
                    item["translation_rms_delta_mm"] = (
                        item["translation_rms_mm"] - baseline["translation_rms_mm"]
                    )
                    item["rotation_rms_delta_deg"] = (
                        item["rotation_rms_deg"] - baseline["rotation_rms_deg"]
                    )
                leave_one_out.append(item)
            except Exception as exc:
                leave_one_out.append(
                    {
                        "removed_sample_id": sample.get("sample_id", index + 1),
                        "removed_index": index,
                        "success": False,
                        "message": str(exc),
                    }
                )
    leave_one_out.sort(key=_leave_one_out_key)
    if max_leave_one_out:
        leave_one_out = leave_one_out[: int(max_leave_one_out)]

    return {
        "sample_count": len(samples),
        "baseline": baseline,
        "best_method": best_method,
        "methods": method_reports,
        "leave_one_out": leave_one_out,
    }


def _summary_from_validation(report):
    return {
        "success": True,
        "sample_count": report["sample_count"],
        "translation_rms_mm": report["translation_rms_mm"],
        "translation_max_mm": report["translation_max_mm"],
        "rotation_rms_deg": report["rotation_rms_deg"],
        "rotation_max_deg": report["rotation_max_deg"],
        "worst_sample_id": report["worst_sample_id"],
    }


def _score_report(report):
    return (
        float(report["translation_rms_mm"]),
        float(report["rotation_rms_deg"]),
        float(report["translation_max_mm"]),
    )


def _leave_one_out_key(report):
    if not report.get("success", True):
        return (1, float("inf"), float("inf"))
    return (0, float(report["translation_rms_mm"]), float(report["rotation_rms_deg"]))


def default_diagnose_file(dataset, diagnose_file):
    if diagnose_file:
        return diagnose_file
    return str(Path(dataset) / "diagnose.yaml")


def save_diagnose_report(path, report):
    Path(path).write_text(yaml.safe_dump(report, sort_keys=False))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Diagnose Dobot hand-eye samples")
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--diagnose-file", default=None)
    parser.add_argument("--max-leave-one-out", type=int, default=0)
    args, _ = parser.parse_known_args(argv)

    dataset = Path(args.dataset)
    samples = load_samples(dataset / "samples")
    report = diagnose_handeye_samples(
        samples,
        max_leave_one_out=args.max_leave_one_out or None,
    )
    diagnose_file = default_diagnose_file(dataset, args.diagnose_file)
    save_diagnose_report(diagnose_file, report)
    print(yaml.safe_dump(report, sort_keys=False))
    print(f"diagnose_file: {diagnose_file}")


if __name__ == "__main__":
    main()
