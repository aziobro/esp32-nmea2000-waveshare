import json

import numpy as np
import pytest

from calibration_json import CalibrationJsonError, build_calibration_doc, identity_doc, write_calibration_json


def test_build_calibration_doc_happy_path():
    doc = build_calibration_doc(
        bias=[12.5, -7.25, 1.0],
        matrix=np.array([[1.1, 0, 0], [0, 0.95, 0], [0, 0, 1.0]]),
        reference_magnitude=50.0,
        quality=0.9,
        orientation=3,
    )
    assert doc["schemaVersion"] == 1
    assert doc["orientation"] == 3
    assert doc["magCalibration"]["valid"] is True
    assert doc["magCalibration"]["bias"] == [12.5, -7.25, 1.0]
    assert doc["magCalibration"]["matrix"][0] == [1.1, 0.0, 0.0]
    assert doc["gyroCalibration"]["valid"] is False
    assert doc["deviationTable"]["enabled"] is False
    assert doc["deviationTable"]["entries"] == []


def test_identity_doc_is_valid_and_marked_low_quality():
    doc = identity_doc()
    assert doc["magCalibration"]["matrix"] == [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    assert doc["magCalibration"]["bias"] == [0.0, 0.0, 0.0]
    assert doc["magCalibration"]["quality"] == 0.0


def test_rejects_bias_out_of_range():
    with pytest.raises(CalibrationJsonError, match="bias"):
        build_calibration_doc(bias=[9999.0, 0, 0], matrix=np.eye(3), reference_magnitude=50.0)


def test_rejects_singular_matrix():
    with pytest.raises(CalibrationJsonError, match="singular"):
        build_calibration_doc(bias=[0, 0, 0], matrix=np.zeros((3, 3)), reference_magnitude=50.0)


def test_rejects_implausibly_scaled_matrix():
    with pytest.raises(CalibrationJsonError, match="singular"):
        build_calibration_doc(bias=[0, 0, 0], matrix=np.diag([9.0, 9.0, 9.0]), reference_magnitude=50.0)


def test_rejects_orientation_out_of_range():
    with pytest.raises(CalibrationJsonError, match="orientation"):
        build_calibration_doc(bias=[0, 0, 0], matrix=np.eye(3), reference_magnitude=50.0, orientation=99)


def test_write_calibration_json_round_trips(tmp_path):
    doc = build_calibration_doc(bias=[1, 2, 3], matrix=np.eye(3), reference_magnitude=48.0)
    path = tmp_path / "cal.json"
    write_calibration_json(str(path), doc)
    with open(path) as f:
        loaded = json.load(f)
    assert loaded == doc
