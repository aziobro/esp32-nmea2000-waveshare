"""Data-quality report (text) and diagnostic plots (matplotlib, optional).

Plotting is behind a lazy import specifically so `calibrate.py` and the
core fit/export path still work on a machine without matplotlib installed
(e.g. a minimal CI runner) - only report generation degrades, with a clear
message, not a hard failure.
"""
from __future__ import annotations

import os
from typing import List, Optional

import numpy as np

from csv_loader import CaptureData
from ellipsoid_fit import EllipsoidFitResult
from quality import QualityIssue


def _matplotlib_pyplot():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        return None


def generate_plots(data: CaptureData, fit_result: Optional[EllipsoidFitResult], output_dir: str) -> List[str]:
    plt = _matplotlib_pyplot()
    if plt is None:
        return []

    os.makedirs(output_dir, exist_ok=True)
    written = []

    raw = data.mag_boat
    corrected = data.mag_corrected
    if fit_result is not None:
        corrected_fitted = (fit_result.matrix @ (raw - fit_result.bias).T).T
    else:
        corrected_fitted = None

    axis_pairs = [("x", "y", 0, 1), ("x", "z", 0, 2), ("y", "z", 1, 2)]
    for label, points, tag in (("raw", raw, "raw"), ("corrected", corrected_fitted if corrected_fitted is not None else corrected, "corrected")):
        for a_name, b_name, ai, bi in axis_pairs:
            fig, ax = plt.subplots(figsize=(5, 5))
            ax.scatter(points[:, ai], points[:, bi], s=6, alpha=0.6)
            ax.set_xlabel(f"mag_{a_name} (uT)")
            ax.set_ylabel(f"mag_{b_name} (uT)")
            ax.set_title(f"Magnetometer {label}: {a_name}{b_name} plane")
            ax.axhline(0, color="gray", linewidth=0.5)
            ax.axvline(0, color="gray", linewidth=0.5)
            ax.set_aspect("equal", adjustable="datalim")
            fig.tight_layout()
            path = os.path.join(output_dir, f"mag_{tag}_{a_name}{b_name}.png")
            fig.savefig(path, dpi=120)
            plt.close(fig)
            written.append(path)

    fig, ax = plt.subplots(figsize=(8, 4))
    t = (data["timestamp_ms"] - data["timestamp_ms"][0]) / 1000.0
    ax.plot(t, data["mag_magnitude"], label="mag_magnitude (raw)", linewidth=1)
    if fit_result is not None:
        fitted_mag = np.linalg.norm(corrected_fitted, axis=1)
        ax.plot(t, fitted_mag, label="magnitude after fitted calibration", linewidth=1)
        ax.axhline(fit_result.reference_magnitude, color="gray", linestyle="--", linewidth=1, label="fitted reference magnitude")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("field magnitude (uT)")
    ax.set_title("Magnetic field magnitude over time")
    ax.legend(fontsize=8)
    fig.tight_layout()
    path = os.path.join(output_dir, "magnitude_over_time.png")
    fig.savefig(path, dpi=120)
    plt.close(fig)
    written.append(path)

    fig, ax = plt.subplots(figsize=(8, 4))
    for col, label in (("dmp_heading_deg", "DMP"), ("compass_heading_deg", "compass"),
                        ("fusion_heading_deg", "fusion"), ("output_heading_deg", "output")):
        ax.plot(t, data[col], label=label, linewidth=1)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("heading (deg)")
    ax.set_title("Heading source comparison")
    ax.legend(fontsize=8)
    fig.tight_layout()
    path = os.path.join(output_dir, "heading_source_comparison.png")
    fig.savefig(path, dpi=120)
    plt.close(fig)
    written.append(path)

    return written


def generate_data_quality_report(
    data: CaptureData,
    fit_result: Optional[EllipsoidFitResult],
    issues: List[QualityIssue],
    plot_paths: List[str],
    source_path: str,
    plots_skipped_reason: Optional[str] = None,
) -> str:
    lines = []
    lines.append(f"# ICM-20948 calibration data-quality report")
    lines.append("")
    lines.append(f"Source capture: `{source_path}`")
    lines.append(f"Samples: {data.row_count}")
    duration_s = (data['timestamp_ms'][-1] - data['timestamp_ms'][0]) / 1000.0
    lines.append(f"Capture duration: {duration_s:.1f} s")
    lines.append(f"FIFO error count (last sample): {int(data['fifo_error_count'][-1])}")
    lines.append(f"Sensor error count (last sample): {int(data['sensor_error_count'][-1])}")
    lines.append("")

    lines.append("## Quality issues")
    if not issues:
        lines.append("No issues detected.")
    else:
        for issue in issues:
            marker = "ERROR" if issue.severity == "error" else "WARNING"
            lines.append(f"- **{marker}** [`{issue.code}`] {issue.message}")
    lines.append("")

    lines.append("## Ellipsoid fit")
    if fit_result is None:
        lines.append("Fit did not succeed - see errors above. No calibration was produced.")
    else:
        b = fit_result.bias
        m = fit_result.matrix
        rms_residual = float(np.sqrt(np.mean(fit_result.residuals ** 2)))
        max_residual = float(np.max(np.abs(fit_result.residuals)))
        lines.append(f"- Hard-iron bias (uT): [{b[0]:.3f}, {b[1]:.3f}, {b[2]:.3f}]")
        lines.append("- Soft-iron matrix:")
        for row in m:
            lines.append(f"  - [{row[0]:.4f}, {row[1]:.4f}, {row[2]:.4f}]")
        lines.append(f"- Fitted reference magnitude: {fit_result.reference_magnitude:.3f} uT")
        lines.append(f"- Fitted ellipsoid radii (pre-normalization): {fit_result.radii}")
        lines.append(f"- RMS residual: {rms_residual:.3f} uT ({rms_residual / fit_result.reference_magnitude * 100:.1f}% of reference magnitude)")
        lines.append(f"- Max residual: {max_residual:.3f} uT")
    lines.append("")

    if plot_paths:
        lines.append("## Plots")
        for p in plot_paths:
            lines.append(f"- `{os.path.basename(p)}`")
    else:
        lines.append("## Plots")
        lines.append(plots_skipped_reason or "matplotlib not installed - plots skipped (fit/report text is unaffected).")
    lines.append("")

    return "\n".join(lines)
