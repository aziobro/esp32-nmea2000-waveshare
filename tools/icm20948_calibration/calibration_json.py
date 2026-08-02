"""Builds calibration JSON in the exact schema the firmware's
ImuCalibrationJson::importJson accepts (lib/icm20948pure/ImuCalibrationJson.cpp).
Keep the two in sync - this module intentionally mirrors that file's field
names and validation bounds so a file written here is guaranteed importable
on-device, and a fit that wouldn't pass firmware validation is caught here
too instead of only failing on import.
"""
from __future__ import annotations

import json
from typing import List, Optional, Sequence

import numpy as np

SCHEMA_VERSION = 1

# Mirrors the bounds importJson() enforces - kept here so a bad fit is
# rejected locally with a clear message instead of producing a JSON file
# that only fails once uploaded to the device.
_BIAS_RANGE = (-2000.0, 2000.0)
_MATRIX_RANGE = (-10.0, 10.0)
_DETERMINANT_MIN = 0.01
_DETERMINANT_MAX = 50.0


class CalibrationJsonError(Exception):
    pass


def _check_bounds(values, lo, hi, label):
    for v in np.asarray(values).ravel():
        if not np.isfinite(v):
            raise CalibrationJsonError(f"{label} contains a non-finite value ({v})")
        if not (lo <= v <= hi):
            raise CalibrationJsonError(f"{label} value {v} is outside firmware-accepted range [{lo}, {hi}]")


def build_calibration_doc(
    bias: Sequence[float],
    matrix: np.ndarray,
    reference_magnitude: float,
    quality: float = 1.0,
    orientation: int = 0,
    gyro_bias: Optional[Sequence[float]] = None,
    gyro_std: Optional[Sequence[float]] = None,
    fixed_heading_offset_deg: float = 0.0,
    deviation_entries: Optional[List[dict]] = None,
    deviation_enabled: bool = False,
) -> dict:
    bias = list(float(x) for x in bias)
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise CalibrationJsonError(f"matrix must be 3x3, got shape {matrix.shape}")

    _check_bounds(bias, *_BIAS_RANGE, "magCalibration.bias")
    _check_bounds(matrix, *_MATRIX_RANGE, "magCalibration.matrix")

    det = float(np.linalg.det(matrix))
    if not np.isfinite(det) or abs(det) < _DETERMINANT_MIN or abs(det) > _DETERMINANT_MAX:
        raise CalibrationJsonError(
            f"magCalibration.matrix is singular or implausibly scaled (determinant {det:.4f}); "
            "firmware would reject this - refusing to write a calibration file that can't be imported"
        )

    if not (0 <= orientation <= 23):
        raise CalibrationJsonError(f"orientation {orientation} out of range [0, 23]")

    doc = {
        "schemaVersion": SCHEMA_VERSION,
        "orientation": int(orientation),
        "magCalibration": {
            "valid": True,
            "quality": float(quality),
            "referenceMagnitude": float(reference_magnitude),
            "bias": bias,
            "matrix": [[float(matrix[i][j]) for j in range(3)] for i in range(3)],
        },
        "gyroCalibration": {
            "valid": gyro_bias is not None,
            "bias": [float(x) for x in gyro_bias] if gyro_bias is not None else [0.0, 0.0, 0.0],
            "standardDeviation": [float(x) for x in gyro_std] if gyro_std is not None else [0.0, 0.0, 0.0],
        },
        "heading": {
            "fixedOffsetDeg": float(fixed_heading_offset_deg),
        },
        "deviationTable": {
            "enabled": bool(deviation_enabled),
            "entries": deviation_entries or [],
        },
    }
    return doc


def write_calibration_json(path: str, doc: dict) -> None:
    with open(path, "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")


def identity_doc() -> dict:
    return build_calibration_doc(bias=[0.0, 0.0, 0.0], matrix=np.eye(3), reference_magnitude=0.0, quality=0.0)
