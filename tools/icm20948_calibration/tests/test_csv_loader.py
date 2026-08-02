import numpy as np
import pytest

import synthetic
from csv_loader import CalibrationDataError, EXPECTED_COLUMNS, load_csv


def test_round_trips_synthetic_capture(tmp_path):
    dataset = synthetic.ideal_sphere(n=50)
    path = tmp_path / "capture.csv"
    synthetic.write_capture_csv(str(path), dataset)

    data = load_csv(str(path))
    assert data.row_count == 50
    np.testing.assert_allclose(data.mag_boat, dataset.points, atol=1e-3)
    assert set(data.columns.keys()) == set(EXPECTED_COLUMNS)


def test_missing_columns_rejected(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text("timestamp_ms,sample_sequence\n1,2\n")
    with pytest.raises(CalibrationDataError, match="missing columns"):
        load_csv(str(path))


def test_wrong_field_count_rejected(tmp_path):
    dataset = synthetic.ideal_sphere(n=5)
    path = tmp_path / "capture.csv"
    synthetic.write_capture_csv(str(path), dataset)
    # truncate the last data line to break its field count
    lines = path.read_text().splitlines()
    lines[-1] = lines[-1].rsplit(",", 5)[0]
    path.write_text("\n".join(lines) + "\n")
    with pytest.raises(CalibrationDataError, match="expected .* fields"):
        load_csv(str(path))


def test_non_numeric_value_rejected(tmp_path):
    dataset = synthetic.ideal_sphere(n=5)
    path = tmp_path / "capture.csv"
    synthetic.write_capture_csv(str(path), dataset)
    text = path.read_text()
    lines = text.splitlines()
    fields = lines[1].split(",")
    fields[2] = "not_a_number"
    lines[1] = ",".join(fields)
    path.write_text("\n".join(lines) + "\n")
    with pytest.raises(CalibrationDataError, match="non-numeric"):
        load_csv(str(path))


def test_empty_file_rejected(tmp_path):
    path = tmp_path / "empty.csv"
    path.write_text("")
    with pytest.raises(CalibrationDataError, match="empty"):
        load_csv(str(path))


def test_header_only_rejected(tmp_path):
    path = tmp_path / "header_only.csv"
    path.write_text(",".join(EXPECTED_COLUMNS) + "\n")
    with pytest.raises(CalibrationDataError, match="no data rows"):
        load_csv(str(path))


def test_corrupt_csv_fixture_rejected(tmp_path):
    path = tmp_path / "corrupt.csv"
    synthetic.write_corrupt_csv(str(path))
    with pytest.raises(CalibrationDataError):
        load_csv(str(path))
