"""Data-quality checks for a magnetometer calibration capture.

These exist so the tool never silently turns a bad capture into a
calibration file - every check below can produce an "error"-severity
issue that the CLI treats as a hard stop (no calibration.json written)
unless the check is only a "warning" (calibration still produced, but the
report should make the caller think twice before trusting it).
"""
from __future__ import annotations

import dataclasses
from typing import List, Optional

import numpy as np

MIN_SAMPLES_FOR_FIT = 9          # ellipsoid_fit's own hard minimum
MIN_SAMPLES_RECOMMENDED = 200    # below this, a fit may succeed but is unreliable
MIN_STATIC_SPREAD_UT = 2.0       # centered points with a smaller spread than this look "stuck"
NEAR_PLANAR_RATIO = 8.0          # max/min PCA singular value ratio considered "too flat"
MIN_SECTOR_COVERAGE_FRACTION = 0.5   # fraction of the 26-sector shell that must have data
IMPLAUSIBLE_MAGNITUDE_MIN_UT = 10.0
IMPLAUSIBLE_MAGNITUDE_MAX_UT = 150.0
EXCESSIVE_OUTLIER_FRACTION = 0.15    # fraction of samples >3 sigma from the fit
EXCESSIVE_RESIDUAL_RATIO = 0.15      # rms residual / reference magnitude


@dataclasses.dataclass
class QualityIssue:
    code: str
    severity: str  # "error" | "warning"
    message: str


def _sector_coverage(points: np.ndarray, bias: Optional[np.ndarray]) -> float:
    """Buckets the (centered) sample directions into the 26 neighbor
    directions of a 3x3x3 grid (the simplest reasonably discriminating
    orientation-coverage grid) and returns the fraction of buckets hit.
    A real boat-swing/tumble calibration should touch most of these; a
    capture that only ever yaws at one attitude will light up a thin ring.
    """
    center = bias if bias is not None else points.mean(axis=0)
    centered = points - center
    norms = np.linalg.norm(centered, axis=1)
    valid = norms > 1e-6
    if not np.any(valid):
        return 0.0
    directions = centered[valid] / norms[valid, None]
    buckets = set()
    for d in directions:
        key = tuple(np.clip(np.round(d), -1, 1).astype(int))
        if key != (0, 0, 0):
            buckets.add(key)
    return len(buckets) / 26.0


def assess_quality(points: np.ndarray, fit_result, fit_error: Optional[Exception]) -> List[QualityIssue]:
    """`points` is the raw Nx3 sample set that was offered to the fit.
    `fit_result` is an EllipsoidFitResult if fitting succeeded, else None
    and `fit_error` holds the exception the fit raised.
    """
    issues: List[QualityIssue] = []
    n = points.shape[0]

    if n < MIN_SAMPLES_FOR_FIT:
        issues.append(QualityIssue("too_few_samples", "error",
                                    f"only {n} samples captured, need at least {MIN_SAMPLES_FOR_FIT} to fit an ellipsoid at all"))
    elif n < MIN_SAMPLES_RECOMMENDED:
        issues.append(QualityIssue("too_few_samples", "warning",
                                    f"only {n} samples captured; {MIN_SAMPLES_RECOMMENDED}+ spread across many orientations is recommended for a reliable fit"))

    spread = points.max(axis=0) - points.min(axis=0)
    if np.all(spread < MIN_STATIC_SPREAD_UT):
        issues.append(QualityIssue("duplicate_static", "error",
                                    f"samples barely vary (per-axis spread {spread} uT) - the sensor was not moved during capture"))

    if n >= 3:
        centered = points - points.mean(axis=0)
        try:
            singular_values = np.linalg.svd(centered, compute_uv=False)
            if singular_values[-1] > 1e-9:
                ratio = singular_values[0] / singular_values[-1]
                if ratio > NEAR_PLANAR_RATIO:
                    issues.append(QualityIssue("near_planar", "error" if fit_result is None else "warning",
                                                f"sample coverage is nearly planar (PCA singular value ratio {ratio:.1f}) - "
                                                "rotate/tilt through more orientations, not just heading sweeps at one attitude"))
            else:
                issues.append(QualityIssue("near_planar", "error",
                                            "sample coverage collapses to a plane or line (a PCA axis has ~zero spread)"))
        except np.linalg.LinAlgError:
            pass

    if fit_result is not None:
        coverage = _sector_coverage(points, fit_result.bias)
        if coverage < MIN_SECTOR_COVERAGE_FRACTION:
            issues.append(QualityIssue("poor_coverage", "warning",
                                        f"only {coverage * 100:.0f}% of orientation sectors have samples "
                                        f"(recommend >= {MIN_SECTOR_COVERAGE_FRACTION * 100:.0f}%) - swing through more headings and tilt angles"))

        rm = fit_result.reference_magnitude
        if not (IMPLAUSIBLE_MAGNITUDE_MIN_UT <= rm <= IMPLAUSIBLE_MAGNITUDE_MAX_UT):
            issues.append(QualityIssue("implausible_magnitude", "error",
                                        f"fitted reference magnitude {rm:.1f} uT is outside the plausible range "
                                        f"[{IMPLAUSIBLE_MAGNITUDE_MIN_UT}, {IMPLAUSIBLE_MAGNITUDE_MAX_UT}] uT for Earth's field - "
                                        "this fit is not trustworthy"))

        residuals = fit_result.residuals
        rms_residual = float(np.sqrt(np.mean(residuals ** 2)))
        if rm > 1e-6 and (rms_residual / rm) > EXCESSIVE_RESIDUAL_RATIO:
            issues.append(QualityIssue("excessive_residual", "error",
                                        f"rms residual {rms_residual:.2f} uT is {rms_residual / rm * 100:.0f}% of the fitted field "
                                        f"magnitude (limit {EXCESSIVE_RESIDUAL_RATIO * 100:.0f}%) - fit does not explain the data well"))

        std = np.std(residuals)
        if std > 1e-9:
            outlier_fraction = float(np.mean(np.abs(residuals) > 3 * std))
            if outlier_fraction > EXCESSIVE_OUTLIER_FRACTION:
                issues.append(QualityIssue("excessive_outliers", "error",
                                            f"{outlier_fraction * 100:.0f}% of samples are >3 sigma outliers from the fit "
                                            f"(limit {EXCESSIVE_OUTLIER_FRACTION * 100:.0f}%) - consider --robust outlier rejection or recapture"))
    elif fit_error is not None:
        issues.append(QualityIssue("singular_fit", "error", str(fit_error)))

    return issues


def has_blocking_issues(issues: List[QualityIssue]) -> bool:
    return any(i.severity == "error" for i in issues)
