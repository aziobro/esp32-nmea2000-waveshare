#!/usr/bin/env python3
"""Offline magnetometer calibration tool for the icm20948 heading pipeline.

Usage:
    python calibrate.py capture.csv --output calibration.json
    python calibrate.py capture.csv --analysis-only

Fits a hard-iron/soft-iron calibration from a runtime CSV capture (see
lib/icm20948pure/ImuDiagnostics.cpp for the capture format, produced
on-device by the Phase 2 logging system), reports on data quality, and -
only if the data clears every quality gate - writes a calibration.json in
the schema lib/icm20948pure/ImuCalibrationJson.cpp accepts for import.

This tool never silently produces a calibration from poor data: any
error-severity quality issue (too few samples, near-planar coverage, a
singular/implausible fit, excessive residual or outliers, an implausible
field magnitude) blocks writing --output unless --force is given, and
--force always prints a loud warning explaining exactly what's being
overridden.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

from calibration_json import CalibrationJsonError, build_calibration_doc, write_calibration_json
from csv_loader import CalibrationDataError, load_csv
from ellipsoid_fit import EllipsoidFitError, fit_ellipsoid, reject_outliers
from quality import assess_quality, has_blocking_issues
from report import generate_data_quality_report, generate_plots


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Fit an icm20948 magnetometer calibration from a runtime CSV capture.")
    parser.add_argument("capture_csv", help="Path to a CSV capture from the firmware's runtime logger.")
    parser.add_argument("--output", help="Path to write calibration.json to. Required unless --analysis-only.")
    parser.add_argument("--analysis-only", action="store_true", help="Only produce the data-quality report/plots; never write a calibration file.")
    parser.add_argument("--report-dir", help="Directory to write the report and plots into (default: alongside --output, or ./<capture-basename>_report/ in analysis-only mode).")
    parser.add_argument("--no-plots", action="store_true", help="Skip plot generation even if matplotlib is available.")
    parser.add_argument("--robust", action="store_true", help="Iteratively reject >3-sigma outliers before the final fit (see ellipsoid_fit.reject_outliers). Never on by default - silent rejection can hide a genuinely bad capture.")
    parser.add_argument("--orientation", type=int, default=0, help="MountOrientation enum value (0-23) to record in the output JSON (default 0 = Forward). Matches lib/icm20948pure/ImuTypes.h.")
    parser.add_argument("--force", action="store_true", help="Write --output even if quality gates failed. Prints a loud warning; use only if you understand and accept the reported issues.")
    parser.add_argument("--reference-magnitude", type=float, default=None,
                         help="Known local magnetic field strength in uT (e.g. from NOAA's online calculator). "
                              "Without this, the fit normalizes to the geometric mean of the fitted ellipsoid's own "
                              "axes, which is self-consistent but an otherwise arbitrary scale (heading only depends "
                              "on direction, so this does not affect heading accuracy either way).")
    args = parser.parse_args(argv)

    if not args.analysis_only and not args.output:
        parser.error("--output is required unless --analysis-only is given")

    try:
        data = load_csv(args.capture_csv)
    except CalibrationDataError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    points = data.mag_boat
    dropped = 0
    fit_result = None
    fit_error = None
    if args.robust:
        try:
            kept_points, mask, fit_result = reject_outliers(points, reference_magnitude=args.reference_magnitude)
            dropped = int((~mask).sum())
            points_for_quality = points  # assess quality against the FULL set so outlier stats are honest
        except EllipsoidFitError as e:
            fit_error = e
            points_for_quality = points
    else:
        try:
            fit_result = fit_ellipsoid(points, reference_magnitude=args.reference_magnitude)
        except EllipsoidFitError as e:
            fit_error = e
        points_for_quality = points

    issues = assess_quality(points_for_quality, fit_result, fit_error)
    blocking = has_blocking_issues(issues)

    report_dir = args.report_dir
    if report_dir is None:
        base = os.path.splitext(os.path.basename(args.capture_csv))[0]
        # Default alongside --output if given, else alongside the input
        # CSV - deliberately never falls back to the current working
        # directory, so running the tool from an arbitrary cwd (or under
        # a test runner) never scatters report directories outside the
        # capture's own location.
        anchor = args.output if args.output else args.capture_csv
        report_dir = os.path.join(os.path.dirname(os.path.abspath(anchor)), f"{base}_report")
    os.makedirs(report_dir, exist_ok=True)

    plot_paths = [] if args.no_plots else generate_plots(data, fit_result, report_dir)
    plots_skipped_reason = "--no-plots given - plots skipped." if args.no_plots else None
    report_text = generate_data_quality_report(data, fit_result, issues, plot_paths, args.capture_csv, plots_skipped_reason)
    if args.robust:
        report_text += f"\n--robust outlier rejection dropped {dropped}/{points.shape[0]} samples before the reported fit.\n"
    report_path = os.path.join(report_dir, "report.md")
    with open(report_path, "w") as f:
        f.write(report_text)

    print(report_text)
    print(f"(report and plots written to {report_dir})")

    if args.analysis_only:
        return 1 if blocking else 0

    if fit_result is None:
        print(f"\nerror: cannot write calibration.json - the ellipsoid fit did not succeed at all ({fit_error}). "
              "There is no fitted bias/matrix to write, --force cannot override this.", file=sys.stderr)
        return 1

    if blocking and not args.force:
        print("\nrefusing to write calibration.json: quality gates failed (see errors above). "
              "Recapture with better coverage, or pass --force to override (not recommended).", file=sys.stderr)
        return 1

    if blocking and args.force:
        print("\nWARNING: --force given, writing calibration.json despite failed quality gates above. "
              "This calibration is NOT verified good.", file=sys.stderr)

    rms_residual = float(np.sqrt(np.mean(fit_result.residuals ** 2)))
    quality_score = max(0.0, min(1.0, 1.0 - (rms_residual / fit_result.reference_magnitude) / 0.15))

    try:
        doc = build_calibration_doc(
            bias=fit_result.bias,
            matrix=fit_result.matrix,
            reference_magnitude=fit_result.reference_magnitude,
            quality=quality_score,
            orientation=args.orientation,
        )
    except CalibrationJsonError as e:
        print(f"error: fit result is not writable as firmware calibration JSON: {e}", file=sys.stderr)
        return 1

    write_calibration_json(args.output, doc)
    print(f"\nwrote calibration to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
