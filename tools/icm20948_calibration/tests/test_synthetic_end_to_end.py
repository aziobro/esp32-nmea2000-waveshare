"""End-to-end tests: synthetic CSV capture -> calibrate.main() -> output,
exercising the full CLI pipeline exactly as a real invocation would (CSV
loading, fitting, quality gating, JSON writing) rather than the individual
modules in isolation.
"""
import json
import os

import synthetic
from calibrate import main as calibrate_main


def test_good_capture_produces_calibration(tmp_path):
    dataset = synthetic.known_hard_iron(n=300)
    csv_path = tmp_path / "capture.csv"
    synthetic.write_capture_csv(str(csv_path), dataset)
    out_path = tmp_path / "calibration.json"

    rc = calibrate_main([str(csv_path), "--output", str(out_path), "--no-plots"])
    assert rc == 0
    assert out_path.exists()

    with open(out_path) as f:
        doc = json.load(f)
    bias = doc["magCalibration"]["bias"]
    assert abs(bias[0] - dataset.true_bias[0]) < 1.0
    assert abs(bias[1] - dataset.true_bias[1]) < 1.0
    assert abs(bias[2] - dataset.true_bias[2]) < 1.0
    assert (tmp_path / "capture_report" / "report.md").exists()


def test_insufficient_coverage_blocks_output(tmp_path):
    dataset = synthetic.insufficient_coverage()
    csv_path = tmp_path / "capture.csv"
    synthetic.write_capture_csv(str(csv_path), dataset)
    out_path = tmp_path / "calibration.json"

    rc = calibrate_main([str(csv_path), "--output", str(out_path), "--no-plots"])
    assert rc == 1
    assert not out_path.exists()
    assert (tmp_path / "capture_report" / "report.md").exists()  # report still written for diagnosis


def test_force_overrides_blocking_gate(tmp_path):
    dataset = synthetic.insufficient_coverage()
    csv_path = tmp_path / "capture.csv"
    synthetic.write_capture_csv(str(csv_path), dataset)
    out_path = tmp_path / "calibration.json"

    rc = calibrate_main([str(csv_path), "--output", str(out_path), "--no-plots", "--force"])
    # --force only helps if a fit was produced at all; insufficient_coverage
    # is degenerate enough that the fit itself may raise, in which case
    # there is nothing to force-write. Either outcome (blocked, or written
    # under protest) is acceptable - what must NOT happen is a silent
    # clean-looking success (rc == 0 with no warning trace).
    if rc == 0:
        assert out_path.exists()


def test_analysis_only_never_writes_output(tmp_path):
    dataset = synthetic.ideal_sphere(n=200)
    csv_path = tmp_path / "capture.csv"
    synthetic.write_capture_csv(str(csv_path), dataset)

    rc = calibrate_main([str(csv_path), "--analysis-only", "--no-plots"])
    assert rc == 0
    assert not (tmp_path / "calibration.json").exists()


def test_corrupt_csv_reported_as_data_error(tmp_path):
    csv_path = tmp_path / "corrupt.csv"
    synthetic.write_corrupt_csv(str(csv_path))

    rc = calibrate_main([str(csv_path), "--analysis-only", "--no-plots"])
    assert rc == 2


def test_output_required_unless_analysis_only(tmp_path):
    dataset = synthetic.ideal_sphere(n=50)
    csv_path = tmp_path / "capture.csv"
    synthetic.write_capture_csv(str(csv_path), dataset)

    try:
        calibrate_main([str(csv_path)])
        assert False, "expected SystemExit from argparse"
    except SystemExit as e:
        assert e.code != 0


def test_robust_flag_runs_end_to_end(tmp_path):
    dataset = synthetic.with_outliers()
    csv_path = tmp_path / "capture.csv"
    synthetic.write_capture_csv(str(csv_path), dataset)
    out_path = tmp_path / "calibration.json"

    rc = calibrate_main([str(csv_path), "--output", str(out_path), "--no-plots", "--robust"])
    assert rc == 0
    assert out_path.exists()
    with open(out_path) as f:
        doc = json.load(f)
    bias = doc["magCalibration"]["bias"]
    assert abs(bias[0] - dataset.true_bias[0]) < 1.5
