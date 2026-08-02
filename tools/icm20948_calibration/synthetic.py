"""Synthetic magnetometer dataset generator for testing the calibration
tool without a physical device.

Each scenario returns raw (uncalibrated, boat-frame) sample points plus
the ground truth (bias, matrix, reference_magnitude) that generated them,
so tests can assert the fit recovers known values within a documented
tolerance (see tests/test_ellipsoid_fit.py and
tests/test_synthetic_end_to_end.py). Scenarios that are deliberately bad
data (insufficient_coverage, flat_only_rotation, duplicate_static,
outliers) are meant to be rejected or flagged by quality.py, not fit
accurately - their tests assert rejection/flagging, not recovery
accuracy.
"""
from __future__ import annotations

import dataclasses
from typing import Optional

import numpy as np

EARTH_FIELD_UT = 50.0  # plausible mid-latitude total field magnitude


@dataclasses.dataclass
class SyntheticDataset:
    name: str
    points: np.ndarray            # Nx3 raw (boat-frame) magnetometer samples
    true_bias: np.ndarray
    true_matrix: np.ndarray       # the matrix a PERFECT fit would recover
    true_reference_magnitude: float
    description: str


def _fibonacci_sphere(n: int) -> np.ndarray:
    """N points spread near-uniformly over the unit sphere - a simple,
    deterministic stand-in for "swing the boat through many orientations"."""
    i = np.arange(n)
    phi = np.arccos(1 - 2 * (i + 0.5) / n)
    golden_angle = np.pi * (3 - np.sqrt(5))
    theta = golden_angle * i
    x = np.sin(phi) * np.cos(theta)
    y = np.sin(phi) * np.sin(theta)
    z = np.cos(phi)
    return np.column_stack([x, y, z])


def _apply_distortion(directions: np.ndarray, bias: np.ndarray, matrix: np.ndarray, magnitude: float) -> np.ndarray:
    """raw = bias + matrix^-1 @ (magnitude * direction) - the inverse of the
    firmware's `corrected = matrix @ (raw - bias)`, so fitting the
    generated raw points should recover `bias` and `matrix`."""
    field = magnitude * directions
    return bias[None, :] + (np.linalg.inv(matrix) @ field.T).T


def ideal_sphere(n: int = 400, seed: int = 0) -> SyntheticDataset:
    directions = _fibonacci_sphere(n)
    bias = np.zeros(3)
    matrix = np.eye(3)
    points = _apply_distortion(directions, bias, matrix, EARTH_FIELD_UT)
    return SyntheticDataset("ideal_sphere", points, bias, matrix, EARTH_FIELD_UT,
                             "Perfect sphere, uniform coverage, no distortion, no noise.")


def known_hard_iron(n: int = 400, seed: int = 0) -> SyntheticDataset:
    directions = _fibonacci_sphere(n)
    bias = np.array([15.0, -8.0, 5.0])
    matrix = np.eye(3)
    points = _apply_distortion(directions, bias, matrix, EARTH_FIELD_UT)
    return SyntheticDataset("known_hard_iron", points, bias, matrix, EARTH_FIELD_UT,
                             "Sphere offset by a fixed hard-iron bias, no soft-iron distortion.")


def unequal_scale(n: int = 400, seed: int = 0) -> SyntheticDataset:
    directions = _fibonacci_sphere(n)
    bias = np.array([5.0, -3.0, 2.0])
    matrix = np.diag([1.2, 0.9, 1.05])
    points = _apply_distortion(directions, bias, matrix, EARTH_FIELD_UT)
    return SyntheticDataset("unequal_scale", points, bias, matrix, EARTH_FIELD_UT,
                             "Axis-aligned soft-iron scaling (no rotation) plus a hard-iron bias.")


def rotated_ellipsoid(n: int = 400, seed: int = 0) -> SyntheticDataset:
    directions = _fibonacci_sphere(n)
    bias = np.array([-10.0, 6.0, -4.0])
    ax, ay, az = np.deg2rad(31), np.deg2rad(-17), np.deg2rad(52)
    Rx = np.array([[1, 0, 0], [0, np.cos(ax), -np.sin(ax)], [0, np.sin(ax), np.cos(ax)]])
    Ry = np.array([[np.cos(ay), 0, np.sin(ay)], [0, 1, 0], [-np.sin(ay), 0, np.cos(ay)]])
    Rz = np.array([[np.cos(az), -np.sin(az), 0], [np.sin(az), np.cos(az), 0], [0, 0, 1]])
    rot = Rz @ Ry @ Rx
    scales = np.diag([1.3, 0.85, 1.1])
    matrix = rot @ scales @ rot.T
    points = _apply_distortion(directions, bias, matrix, EARTH_FIELD_UT)
    return SyntheticDataset("rotated_ellipsoid", points, bias, matrix, EARTH_FIELD_UT,
                             "Rotated, unequal-axis soft-iron ellipsoid plus a hard-iron bias - the general case.")


def noisy(n: int = 400, seed: int = 1, noise_std_ut: float = 0.6) -> SyntheticDataset:
    rng = np.random.default_rng(seed)
    base = known_hard_iron(n)
    noisy_points = base.points + rng.normal(scale=noise_std_ut, size=base.points.shape)
    return SyntheticDataset("noise", noisy_points, base.true_bias, base.true_matrix, base.true_reference_magnitude,
                             f"known_hard_iron with additive Gaussian sensor noise (std={noise_std_ut} uT).")


def with_outliers(n: int = 400, seed: int = 2, n_outliers: int = 25) -> SyntheticDataset:
    rng = np.random.default_rng(seed)
    base = known_hard_iron(n)
    points = base.points.copy()
    idx = rng.choice(n, size=n_outliers, replace=False)
    points[idx] += rng.normal(scale=40.0, size=(n_outliers, 3))
    return SyntheticDataset("outliers", points, base.true_bias, base.true_matrix, base.true_reference_magnitude,
                             f"known_hard_iron with {n_outliers}/{n} samples replaced by large outliers "
                             "(e.g. transient magnetic interference during capture).")


def insufficient_coverage(n: int = 12, seed: int = 3) -> SyntheticDataset:
    rng = np.random.default_rng(seed)
    theta = rng.uniform(0, 2 * np.pi, n)
    directions = np.column_stack([np.cos(theta) * 0.05, np.sin(theta) * 0.05, np.ones(n)])
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    bias = np.zeros(3)
    matrix = np.eye(3)
    points = _apply_distortion(directions, bias, matrix, EARTH_FIELD_UT)
    return SyntheticDataset("insufficient_coverage", points, bias, matrix, EARTH_FIELD_UT,
                             f"Only {n} samples clustered in a narrow cone - too few and too little spread to fit reliably.")


def flat_only_rotation(n: int = 200, seed: int = 4) -> SyntheticDataset:
    theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
    directions = np.column_stack([np.cos(theta), np.sin(theta), np.full(n, 0.02)])
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    bias = np.zeros(3)
    matrix = np.eye(3)
    points = _apply_distortion(directions, bias, matrix, EARTH_FIELD_UT)
    return SyntheticDataset("flat_only_rotation", points, bias, matrix, EARTH_FIELD_UT,
                             "Boat yawed through a full circle but never tilted - all samples near one horizontal plane.")


def duplicate_static(n: int = 200, seed: int = 5) -> SyntheticDataset:
    rng = np.random.default_rng(seed)
    direction = np.array([0.3, 0.1, 0.94])
    direction /= np.linalg.norm(direction)
    jitter = rng.normal(scale=0.02, size=(n, 3))
    points = EARTH_FIELD_UT * direction[None, :] + jitter
    return SyntheticDataset("duplicate_static", points, np.zeros(3), np.eye(3), EARTH_FIELD_UT,
                             "Device sitting still on the bench - all samples nearly identical, never swung.")


SCENARIOS = {
    "ideal_sphere": ideal_sphere,
    "known_hard_iron": known_hard_iron,
    "unequal_scale": unequal_scale,
    "rotated_ellipsoid": rotated_ellipsoid,
    "noise": noisy,
    "outliers": with_outliers,
    "insufficient_coverage": insufficient_coverage,
    "flat_only_rotation": flat_only_rotation,
    "duplicate_static": duplicate_static,
}


def generate(name: str, **kwargs) -> SyntheticDataset:
    if name not in SCENARIOS:
        raise ValueError(f"unknown synthetic scenario '{name}', have: {sorted(SCENARIOS)}")
    return SCENARIOS[name](**kwargs)


def to_capture_csv_rows(dataset: SyntheticDataset):
    """Yields dict rows matching csv_loader.EXPECTED_COLUMNS, wrapping the
    synthetic magnetometer points in a plausible full 41-column capture so
    the tool's CSV loader is exercised end-to-end, not just the raw numpy
    fit. Non-magnetometer fields are filled with realistic placeholder
    values (level attitude, identity quaternion, no errors) since this
    dataset is purely about magnetometer calibration - only mag_boat is
    meaningful."""
    from csv_loader import EXPECTED_COLUMNS  # local import avoids a cycle at module load time

    n = dataset.points.shape[0]
    for i in range(n):
        mx, my, mz = dataset.points[i]
        row = {c: 0 for c in EXPECTED_COLUMNS}
        row["timestamp_ms"] = i * 100
        row["sample_sequence"] = i
        row["accel_raw_x"] = row["accel_boat_x"] = 0.0
        row["accel_raw_y"] = row["accel_boat_y"] = 0.0
        row["accel_raw_z"] = row["accel_boat_z"] = 1.0
        row["mag_raw_x"] = row["mag_boat_x"] = mx
        row["mag_raw_y"] = row["mag_boat_y"] = my
        row["mag_raw_z"] = row["mag_boat_z"] = mz
        row["mag_corrected_x"] = mx
        row["mag_corrected_y"] = my
        row["mag_corrected_z"] = mz
        row["mag_magnitude"] = float(np.linalg.norm(dataset.points[i]))
        row["dmp_q0"] = 1.0
        row["active_heading_source"] = "software_compass"
        row["heading_quality"] = "good"
        yield row


def write_capture_csv(path: str, dataset: SyntheticDataset) -> None:
    import csv as _csv

    from csv_loader import EXPECTED_COLUMNS

    with open(path, "w", newline="") as f:
        writer = _csv.DictWriter(f, fieldnames=EXPECTED_COLUMNS)
        writer.writeheader()
        for row in to_capture_csv_rows(dataset):
            writer.writerow(row)


def write_corrupt_csv(path: str) -> None:
    """A CSV that superficially looks like a capture but is truncated/
    malformed - used to test csv_loader's error handling."""
    with open(path, "w") as f:
        f.write("timestamp_ms,sample_sequence,accel_raw_x\n")
        f.write("this is not,a,valid\n,capture,file\n")
