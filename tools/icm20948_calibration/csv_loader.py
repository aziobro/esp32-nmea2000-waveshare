"""Loads the runtime diagnostic CSV produced by the firmware's
Icm20948Capture (see lib/icm20948pure/ImuDiagnostics.cpp for the
authoritative column list and lib/icm20948task/GwIcm20948CaptureTask.cpp
for how it's captured on-device).

Deliberately does not depend on pandas - this tool's only hard dependency
beyond the standard library is numpy (matplotlib is optional, see
report.py), to keep setup light for an occasional offline diagnostic tool.
"""
from __future__ import annotations

import csv
import dataclasses
from typing import List

import numpy as np

# Must match ImuDiagnostics::csvHeader() exactly, in order.
EXPECTED_COLUMNS = [
    "timestamp_ms", "sample_sequence",
    "accel_raw_x", "accel_raw_y", "accel_raw_z",
    "gyro_raw_x", "gyro_raw_y", "gyro_raw_z",
    "mag_raw_x", "mag_raw_y", "mag_raw_z",
    "accel_boat_x", "accel_boat_y", "accel_boat_z",
    "gyro_boat_x", "gyro_boat_y", "gyro_boat_z",
    "mag_boat_x", "mag_boat_y", "mag_boat_z",
    "mag_corrected_x", "mag_corrected_y", "mag_corrected_z",
    "mag_magnitude",
    "dmp_q0", "dmp_q1", "dmp_q2", "dmp_q3",
    "dmp_roll_deg", "dmp_pitch_deg", "dmp_heading_deg",
    "compass_heading_deg", "fusion_heading_deg", "output_heading_deg",
    "rate_of_turn_deg_s",
    "active_heading_source", "heading_quality", "rejection_flags",
    "dmp_sample_age_ms", "dmp_compass_age_ms",
    "fifo_error_count", "fifo_drain_limit_count", "sensor_error_count",
]

LEGACY_COLUMNS = [
    c for c in EXPECTED_COLUMNS
    if c not in {"dmp_compass_age_ms", "fifo_drain_limit_count"}
]

_STRING_COLUMNS = {"active_heading_source", "heading_quality"}

NUMERIC_COLUMNS = [c for c in EXPECTED_COLUMNS if c not in _STRING_COLUMNS]


class CalibrationDataError(Exception):
    """Raised for CSV that doesn't match the expected capture format."""


@dataclasses.dataclass
class CaptureData:
    columns: dict  # column name -> np.ndarray (numeric) or list[str] (string columns)
    row_count: int

    def __getitem__(self, name: str):
        return self.columns[name]

    def subset(self, mask: np.ndarray) -> "CaptureData":
        if mask.shape[0] != self.row_count:
            raise ValueError("subset mask length does not match row count")
        columns = {}
        for name, values in self.columns.items():
            if name in _STRING_COLUMNS:
                columns[name] = [v for v, keep in zip(values, mask) if keep]
            else:
                columns[name] = values[mask]
        return CaptureData(columns=columns, row_count=int(np.sum(mask)))

    @property
    def mag_boat(self) -> np.ndarray:
        """Nx3 array: magnetometer samples in boat frame, BEFORE bias/matrix
        correction - this is what the firmware's ImuCalibrationOps::applyMag
        is applied to (see lib/icm20948pure/ImuCalibration.cpp), so it's the
        correct input for fitting a new calibration."""
        return np.column_stack([self.columns["mag_boat_x"], self.columns["mag_boat_y"], self.columns["mag_boat_z"]])

    @property
    def mag_corrected(self) -> np.ndarray:
        return np.column_stack([self.columns["mag_corrected_x"], self.columns["mag_corrected_y"], self.columns["mag_corrected_z"]])

    @property
    def mag_raw(self) -> np.ndarray:
        """Nx3 array: effective pre-user-calibration magnetometer source.
        New DMP captures use the DMP compass field here; non-DMP and legacy
        captures use the AGMT register decode."""
        return np.column_stack([self.columns["mag_raw_x"], self.columns["mag_raw_y"], self.columns["mag_raw_z"]])


def load_csv(path: str) -> CaptureData:
    with open(path, "r", newline="") as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            raise CalibrationDataError(f"{path}: file is empty")

        header = [h.strip() for h in header]
        legacy = header == LEGACY_COLUMNS
        if header != EXPECTED_COLUMNS and not legacy:
            missing = [c for c in EXPECTED_COLUMNS if c not in header]
            extra = [c for c in header if c not in EXPECTED_COLUMNS]
            detail = []
            if missing:
                detail.append(f"missing columns: {missing}")
            if extra:
                detail.append(f"unexpected columns: {extra}")
            if not detail:
                detail.append("column order does not match the firmware's csvHeader()")
            raise CalibrationDataError(f"{path}: does not look like an icm20948 capture CSV ({'; '.join(detail)})")

        raw_rows: List[List[str]] = []
        for line_no, row in enumerate(reader, start=2):
            if legacy and len(row) == len(LEGACY_COLUMNS):
                dmp_age = row[LEGACY_COLUMNS.index("dmp_sample_age_ms")]
                row = row[:39] + [dmp_age, row[39], "0", row[40]]
            if len(row) != len(EXPECTED_COLUMNS):
                raise CalibrationDataError(
                    f"{path}:{line_no}: expected {len(EXPECTED_COLUMNS)} fields, got {len(row)} (corrupt or truncated capture)"
                )
            raw_rows.append(row)

    if not raw_rows:
        raise CalibrationDataError(f"{path}: header present but no data rows")

    columns = {}
    for col_idx, name in enumerate(EXPECTED_COLUMNS):
        values = [row[col_idx] for row in raw_rows]
        if name in _STRING_COLUMNS:
            columns[name] = values
        else:
            try:
                columns[name] = np.array([float(v) for v in values], dtype=np.float64)
            except ValueError as e:
                raise CalibrationDataError(f"{path}: column '{name}' contains a non-numeric value ({e})")

    return CaptureData(columns=columns, row_count=len(raw_rows))
