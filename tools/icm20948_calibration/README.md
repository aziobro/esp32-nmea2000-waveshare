# icm20948 offline calibration tool

Fits a magnetometer hard-iron/soft-iron calibration from a runtime CSV
capture produced by the firmware's diagnostic logger (see
[`lib/icm20948pure/ImuDiagnostics.cpp`](../../lib/icm20948pure/ImuDiagnostics.cpp)
for the capture format and [`lib/icm20948task/GwIcm20948CaptureTask.cpp`](../../lib/icm20948task/GwIcm20948CaptureTask.cpp)
for how to capture one on-device), and writes a `calibration.json` in the
schema [`lib/icm20948pure/ImuCalibrationJson.cpp`](../../lib/icm20948pure/ImuCalibrationJson.cpp)
accepts for import.

**This tool never silently produces a calibration from poor data.** Every
run produces a data-quality report; if the capture fails any quality gate
(too few samples, coverage that's near-planar or too narrow, a
singular/implausible fit, excessive residual or outlier fraction, an
implausible fitted field magnitude), `--output` is refused unless you pass
`--force` - and `--force` prints a loud warning, it doesn't silence one.

## Setup

```sh
cd tools/icm20948_calibration
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt              # numpy only - fit/export works headless
./.venv/bin/pip install -r requirements-optional.txt      # + matplotlib, for plots/reports
./.venv/bin/pip install -r requirements-dev.txt           # + pytest, to run the test suite
```

## Usage

```sh
# Fit and write a calibration file:
./.venv/bin/python calibrate.py capture.csv --output calibration.json

# Report-only, no calibration file written (e.g. to check a capture before trusting it):
./.venv/bin/python calibrate.py capture.csv --analysis-only

# If you know the local total field strength (e.g. from NOAA's magnetic
# field calculator), pin the calibration's absolute scale to it - without
# this the tool normalizes to the fitted ellipsoid's own geometric mean,
# which is self-consistent but doesn't matter for heading accuracy either
# way (heading depends only on direction, not field magnitude):
./.venv/bin/python calibrate.py capture.csv --output calibration.json --reference-magnitude 52.3

# Iteratively reject >3-sigma outliers before the final fit (off by
# default - silent rejection can hide a genuinely bad capture):
./.venv/bin/python calibrate.py capture.csv --output calibration.json --robust
```

Every run writes `report.md` plus (if matplotlib is installed) plots into
`--report-dir` (default: `<capture-basename>_report/` next to `--output`).

## What "quality" checks for

See [`quality.py`](quality.py) for exact thresholds. Blocking (`error`)
checks: `too_few_samples` (fit needs >=9, but recommends 200+),
`duplicate_static` (device never moved), `near_planar` (rotated about only
one axis, e.g. yaw-only swings with no tilt), `singular_fit` (the
underlying ellipsoid fit itself failed), `implausible_magnitude` (fitted
field strength outside a plausible Earth-field range), `excessive_residual`,
`excessive_outliers`. `poor_coverage` and small sample counts above the
hard minimum are `warning`-only.

## How the fit works

[`ellipsoid_fit.py`](ellipsoid_fit.py) implements the standard closed-form
general-ellipsoid least-squares fit (the same family of technique used by
most open-source magnetometer calibration tools): fit the general quadric
surface through the sample points, then decompose it via eigendecomposition
into a center (hard-iron bias) and a symmetric matrix (soft-iron
correction), matching the firmware's `corrected = matrix * (raw - bias)`
convention exactly (see `lib/icm20948pure/ImuCalibration.cpp`).

## Testing

```sh
./.venv/bin/python -m pytest tests/ -v
```

Tests run against synthetic datasets ([`synthetic.py`](synthetic.py)) with
documented, known ground truth - no physical device or captured data
needed:

| scenario | what it exercises |
|---|---|
| `ideal_sphere` | Perfect data, no distortion - exact recovery. |
| `known_hard_iron` | Fixed offset only - exact recovery. |
| `unequal_scale` | Axis-aligned soft-iron scaling - exact recovery when reference magnitude is known, shape recovered up to a scalar otherwise. |
| `rotated_ellipsoid` | Rotated + unequal-axis soft iron - the general case. |
| `noise` | Gaussian sensor noise - recovery within a documented loose tolerance. |
| `outliers` | A fraction of samples replaced with large outliers - proves the naive fit is measurably degraded, and that `--robust` recovers close to ground truth. |
| `insufficient_coverage` | Too few samples in too narrow a cone - fit rejected or flagged, never trusted. |
| `flat_only_rotation` | Yawed through a full circle but never tilted - `near_planar`, singular fit. |
| `duplicate_static` | Device never moved - `duplicate_static`, singular fit. |
| `corrupt_csv` (via `write_corrupt_csv`) | Malformed/truncated capture file - `csv_loader` rejection. |

`tests/test_synthetic_end_to_end.py` drives the actual CLI (`calibrate.main`)
against several of these end to end, including the blocking-quality-gate and
`--force` paths. `test/test_calibration_json/test_main.cpp`
(the firmware's native test suite, not part of this Python tool) includes
`test_imports_json_produced_by_the_python_calibration_tool`, which imports a
JSON file captured verbatim from an actual run of this tool through the
real C++ `ImuCalibrationJson::importJson` - a genuine cross-language
compatibility check, not just "the schemas match on paper."

## Sample report

[`docs/sample_report/`](docs/sample_report/) contains a full report
(`report.md`, all plots, the source capture, and the resulting calibration
JSON) generated from the synthetic `rotated_ellipsoid` scenario, committed
as a worked example of the tool's output. **It was generated entirely from
synthetic data - it demonstrates the tool's mechanics, not real-world
calibration accuracy on physical hardware**, which has not yet been
tested.
