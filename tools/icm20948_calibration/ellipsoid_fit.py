"""General ellipsoid least-squares fit for magnetometer hard-iron/soft-iron
calibration.

Implements the standard closed-form algebraic ellipsoid fit (originally
described by Li & Griffiths, "Least Squares Ellipsoid Specific Fitting",
2004; the specific linear-algebra formulation used here follows the widely
used MATLAB `ellipsoid_fit` by Yury Petrov, ported to numpy). This is the
same established technique used by most open-source magnetometer
calibration tools: fit the general quadric surface through the sample
points, then decompose it into a center (hard-iron bias) and a symmetric
matrix (soft-iron correction) via eigendecomposition.

The firmware applies calibration as (see lib/icm20948pure/ImuCalibration.cpp):

    corrected = magMatrix * (raw - magBias)

so `fit_ellipsoid` here returns `bias` and `matrix` in exactly that
convention: for a sample s used in the fit, matrix @ (s - bias) should have
norm close to `reference_magnitude`.
"""
from __future__ import annotations

import dataclasses

import numpy as np


class EllipsoidFitError(Exception):
    """Raised when the sample set is fundamentally unfittable (e.g. too few
    points, or the fit produces a singular/non-ellipsoidal solution)."""


@dataclasses.dataclass
class EllipsoidFitResult:
    bias: np.ndarray          # (3,) hard-iron offset
    matrix: np.ndarray        # (3,3) soft-iron correction matrix
    reference_magnitude: float
    radii: np.ndarray         # (3,) fitted semi-axis lengths, before normalization
    evecs: np.ndarray         # (3,3) fitted ellipsoid axis directions
    residuals: np.ndarray     # (N,) |matrix @ (s - bias)| - reference_magnitude, per sample


def fit_ellipsoid(points: np.ndarray, reference_magnitude: float = None) -> EllipsoidFitResult:
    """Fits a general (rotated, unequal-axis) ellipsoid to `points` (Nx3).

    An ellipsoid fit alone cannot recover the sensor's absolute field
    magnitude - scaling a valid calibration matrix by any positive
    constant produces an equally valid calibration (heading only depends
    on direction, not magnitude). By default this picks the geometric
    mean of the fitted semi-axes as the target sphere radius, which is a
    self-consistent but otherwise arbitrary choice. If the caller knows
    the true field strength (e.g. from a reference magnetometer or a
    published local value), pass `reference_magnitude` to pin the output
    matrix to that exact scale instead.

    Raises EllipsoidFitError if there are too few points or the fit is
    numerically degenerate (singular design matrix, non-positive-definite
    shape matrix, or a radius so small/large it can't be a real ellipsoid).
    Never returns a "best effort" calibration silently - callers should
    treat any successful return as usable, and any exception as "do not
    calibrate from this data".
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise EllipsoidFitError("points must be an Nx3 array")
    n = points.shape[0]
    if n < 9:
        raise EllipsoidFitError(f"need at least 9 points to fit a general ellipsoid, got {n}")

    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    # General quadric a*x^2+b*y^2+c*z^2+2d*xy+2e*xz+2f*yz+2g*x+2h*y+2i*z+j=0
    # is only defined up to an overall nonzero scale, so a normalizing
    # constraint is needed to make the linear system well-posed. Fix
    # a+b+c=3 (the exact constant is arbitrary - the final calibration
    # matrix is re-normalized to reference_magnitude below regardless) and
    # substitute c = 3-a-b, giving:
    #   a*(x^2-z^2) + b*(y^2-z^2) + 2d*xy+2e*xz+2f*yz+2g*x+2h*y+2i*z+j = -3z^2
    # which is a plain linear least-squares problem in (a,b,d,e,f,g,h,i,j).
    P = x * x - z * z
    Q = y * y - z * z
    D = np.array([
        P,
        Q,
        2 * x * y,
        2 * x * z,
        2 * y * z,
        2 * x,
        2 * y,
        2 * z,
        np.ones_like(x),
    ]).T
    target = -3 * z * z

    try:
        DtD = D.T @ D
        if np.linalg.matrix_rank(DtD) < DtD.shape[0]:
            raise EllipsoidFitError("sample points do not constrain a unique ellipsoid (rank-deficient fit - likely too few distinct orientations)")
        w = np.linalg.solve(DtD, D.T @ target)
    except np.linalg.LinAlgError as e:
        raise EllipsoidFitError(f"ellipsoid fit failed to solve (singular system): {e}")

    a, b = w[0], w[1]
    c = 3 - a - b
    v = np.array([a, b, c, w[2], w[3], w[4], w[5], w[6], w[7], w[8]])

    A4 = np.array([
        [v[0], v[3], v[4], v[6]],
        [v[3], v[1], v[5], v[7]],
        [v[4], v[5], v[2], v[8]],
        [v[6], v[7], v[8], v[9]],
    ])
    A3 = A4[:3, :3]

    if abs(np.linalg.det(A3)) < 1e-9:
        raise EllipsoidFitError("fitted quadric is singular (determinant ~0) - not a valid ellipsoid, likely near-planar sample coverage")

    try:
        center = np.linalg.solve(-A3, v[6:9])
    except np.linalg.LinAlgError as e:
        raise EllipsoidFitError(f"could not solve for ellipsoid center: {e}")

    T = np.eye(4)
    T[3, :3] = center
    R = T @ A4 @ T.T

    evals, evecs = np.linalg.eigh(R[:3, :3] / -R[3, 3])
    if np.any(evals <= 0):
        raise EllipsoidFitError("fitted shape is not a real ellipsoid (non-positive eigenvalues) - sample coverage is likely too poor (near-planar or too narrow an arc)")

    radii = np.sqrt(1.0 / evals)
    if np.any(~np.isfinite(radii)) or np.any(radii <= 1e-6) or np.any(radii > 1e6):
        raise EllipsoidFitError(f"fitted ellipsoid radii are implausible ({radii}) - reject rather than trust this fit")

    # Geometric mean preserves the fitted ellipsoid's overall "volume" as
    # the target sphere radius, rather than requiring an externally known
    # field strength (the firmware only cares about heading direction, not
    # absolute magnitude - see the "reserved" referenceMagnitude field in
    # lib/icm20948pure/ImuCalibrationJson.cpp) - used unless the caller
    # supplied a known reference_magnitude to pin the scale exactly.
    if reference_magnitude is None:
        reference_magnitude = float(np.cbrt(radii[0] * radii[1] * radii[2]))

    matrix = evecs @ np.diag(reference_magnitude / radii) @ evecs.T

    corrected = (matrix @ (points - center).T).T
    magnitudes = np.linalg.norm(corrected, axis=1)
    residuals = magnitudes - reference_magnitude

    return EllipsoidFitResult(
        bias=center,
        matrix=matrix,
        reference_magnitude=reference_magnitude,
        radii=radii,
        evecs=evecs,
        residuals=residuals,
    )


def reject_outliers(points: np.ndarray, max_iterations: int = 3, sigma: float = 3.0, reference_magnitude: float = None):
    """Iteratively fits an ellipsoid and drops points whose residual exceeds
    `sigma` standard deviations, re-fitting each round. Returns
    (kept_points, kept_mask, last_fit). Only ever used when the caller
    explicitly opts in (--robust) - silent outlier rejection would hide
    genuinely bad captures rather than surfacing them, which the Phase 4
    spec explicitly warns against ("never silently generate calibration
    from poor data").
    """
    mask = np.ones(points.shape[0], dtype=bool)
    fit = None
    for _ in range(max_iterations):
        fit = fit_ellipsoid(points[mask], reference_magnitude=reference_magnitude)
        full_residuals = np.full(points.shape[0], np.nan)
        full_residuals[mask] = fit.residuals
        std = np.std(fit.residuals)
        if std < 1e-9:
            break
        threshold = sigma * std
        new_mask = mask.copy()
        idx = np.where(mask)[0]
        new_mask[idx[np.abs(fit.residuals) > threshold]] = False
        if new_mask.sum() == mask.sum():
            break
        if new_mask.sum() < 9:
            break
        mask = new_mask
    return points[mask], mask, fit
