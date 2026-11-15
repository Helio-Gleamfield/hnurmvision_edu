"""Independently recompute errors and check delayed-evaluation timestamps.

Only the Python standard library is required. Run from any working directory:
  python3 lesson04/scripts/audit.py --output lesson04/artifacts/audit.json
"""

import argparse
import csv
import json
import math
from decimal import Decimal
from pathlib import Path


def rows(path):
    with path.open(newline="") as stream:
        result = list(csv.DictReader(stream))
    if not result:
        raise ValueError(f"Empty evidence: {path}")
    return result


def statistics(values):
    values = sorted(values)
    assert values and all(math.isfinite(v) and v >= 0 for v in values)
    return {
        "count": len(values),
        "mean": math.fsum(values) / len(values),
        "median_upper": values[len(values) // 2],
        "p95_order_statistic": values[min(len(values) - 1, int(.95 * len(values)))],
        "max": values[-1],
    }


def audit_case(directory, name, horizon_ms):
    poses = rows(directory / f"{name}-poses.csv")
    forecasts = rows(directory / f"{name}-forecasts.csv")
    pose_stamps = {int(row["stamp_ns"]) for row in poses}
    assert len(pose_stamps) == len(poses), "Duplicate pose timestamps"
    assert [int(r["stamp_ns"]) for r in poses] == sorted(pose_stamps)
    pose_errors = []
    for row in poses:
        error = math.sqrt(sum(
            (float(row[f"pnp_{axis}_m"]) - float(row[f"truth_{axis}_m"])) ** 2
            for axis in "xyz"
        ))
        assert abs(error - float(row["position_error_m"])) < 1e-7
        assert 0 <= float(row["reprojection_px"]) <= 3
        pose_errors.append(error)
    prediction_errors = []
    lags = []
    for row in forecasts:
        issued = int(row["issued_ns"])
        due = int(row["due_ns"])
        evaluated = int(row["evaluated_ns"])
        assert issued in pose_stamps, "Forecast lacks a paired observation"
        assert Decimal(row["horizon_ms"]) == Decimal(horizon_ms)
        assert due - issued == int(Decimal(row["horizon_ms"]) * 1_000_000)
        assert evaluated >= due, "Prediction evaluated before its future frame"
        assert 0 < float(row["interpolation_gap_ms"]) <= 100
        lag = (evaluated - due) / 1e6
        assert lag <= float(row["interpolation_gap_ms"]) + 1e-6
        error = math.sqrt(sum(
            (float(row[f"pred_{axis}_m"]) - float(row[f"true_{axis}_m"])) ** 2
            for axis in "xyz"
        ))
        assert abs(error - float(row["error_m"])) < 1e-7
        prediction_errors.append(error)
        lags.append(lag)
    return {
        "status": "PASS",
        "horizon_ms": horizon_ms,
        "pose_error_m": statistics(pose_errors),
        "orientation_error_deg": statistics([float(r["rotation_error_deg"]) for r in poses]),
        "delayed_prediction_error_m": statistics(prediction_errors),
        "bracketing_lag_ms": statistics(lags),
        "checks": [
            "Integer nanosecond timestamps; strictly ordered unique observations",
            "Every saved forecast was issued from a paired observation",
            "due - issued equals the requested horizon exactly",
            "Evaluation timestamp is at or after due timestamp",
            "Truth interpolation bracket is at most 100 ms wide",
            "3D Euclidean position and prediction errors recomputed from CSV",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifacts", type=Path,
                        default=Path(__file__).resolve().parents[1] / "artifacts")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = {name: audit_case(args.artifacts, name, horizon)
              for name, horizon in [("static", 250), ("translation", 250),
                                    ("horizon-100ms", 100)]}
    encoded = json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.write_text(encoded)
    print(encoded, end="")


if __name__ == "__main__":
    main()
