"""Correctness tests for the ellipsoid fit against synthetic ground truth.

Tolerances documented per-scenario below. Noiseless scenarios (ideal
sphere, known hard-iron, unequal scale, rotated ellipsoid) should recover
the ground truth almost exactly - the fit is an exact linear-algebraic
solution for perfect ellipsoid data, so tight tolerances are meaningful
regression protection. The noisy scenario uses a looser tolerance
reflecting that Gaussian sensor noise necessarily perturbs the fit.

Important caveat that shapes several tests below: an ellipsoid fit alone
cannot recover the sensor's absolute field magnitude - any positive
scalar multiple of a correct calibration matrix is an equally valid
calibration (heading depends only on direction). For an isotropic true
matrix (identity, i.e. ideal_sphere/known_hard_iron) the default
geometric-mean normalization happens to land exactly on the true scale.
For anisotropic true matrices (unequal_scale/rotated_ellipsoid) it does
NOT, by construction - those tests pass the dataset's true
reference_magnitude explicitly to pin the scale, matching how a real user
would use --reference-magnitude with a known field strength. A separate
test below (test_unequal_scale_shape_recovered_even_without_reference_magnitude)
confirms the *shape* (bias + matrix up to a scalar) is still recovered
correctly when no reference magnitude is supplied.
"""
import numpy as np
import pytest

import synthetic
from ellipsoid_fit import EllipsoidFitError, fit_ellipsoid, reject_outliers


def _assert_recovers(dataset, bias_tol, matrix_tol, magnitude_tol, reference_magnitude=None):
    result = fit_ellipsoid(dataset.points, reference_magnitude=reference_magnitude)
    np.testing.assert_allclose(result.bias, dataset.true_bias, atol=bias_tol)
    np.testing.assert_allclose(result.matrix, dataset.true_matrix, atol=matrix_tol)
    assert abs(result.reference_magnitude - dataset.true_reference_magnitude) < magnitude_tol
    return result


def test_ideal_sphere_recovered_almost_exactly():
    _assert_recovers(synthetic.ideal_sphere(), bias_tol=1e-6, matrix_tol=1e-6, magnitude_tol=1e-6)


def test_known_hard_iron_recovered_almost_exactly():
    _assert_recovers(synthetic.known_hard_iron(), bias_tol=1e-6, matrix_tol=1e-6, magnitude_tol=1e-6)


def test_unequal_scale_recovered_exactly_with_known_reference_magnitude():
    dataset = synthetic.unequal_scale()
    _assert_recovers(dataset, bias_tol=1e-5, matrix_tol=1e-5, magnitude_tol=1e-5,
                      reference_magnitude=dataset.true_reference_magnitude)


def test_rotated_ellipsoid_recovered_exactly_with_known_reference_magnitude():
    dataset = synthetic.rotated_ellipsoid()
    _assert_recovers(dataset, bias_tol=1e-4, matrix_tol=1e-4, magnitude_tol=1e-4,
                      reference_magnitude=dataset.true_reference_magnitude)


def test_unequal_scale_shape_recovered_even_without_reference_magnitude():
    # Without a known reference magnitude, the fitted matrix should still
    # be proportional to the true matrix by a single positive scalar (the
    # SHAPE - bias, axis directions, and axis ratios - is fully
    # determined by geometry alone; only the absolute scale is not).
    dataset = synthetic.unequal_scale()
    result = fit_ellipsoid(dataset.points)
    np.testing.assert_allclose(result.bias, dataset.true_bias, atol=1e-5)
    ratio_matrix = result.matrix @ np.linalg.inv(dataset.true_matrix)
    scalar = np.trace(ratio_matrix) / 3.0
    np.testing.assert_allclose(ratio_matrix, scalar * np.eye(3), atol=1e-6)
    assert scalar > 0


def test_noisy_data_recovered_within_loose_tolerance():
    # Documented tolerance: with noise_std=0.6 uT over 400 well-distributed
    # samples, hard-iron bias should still land within 2 uT and the
    # reference magnitude within 2 uT of ground truth.
    _assert_recovers(synthetic.noisy(), bias_tol=2.0, matrix_tol=0.1, magnitude_tol=2.0)


def test_outliers_corrupt_a_naive_fit():
    dataset = synthetic.with_outliers()
    result = fit_ellipsoid(dataset.points)
    # With 25/400 samples wildly displaced, a fit that ignores them entirely
    # would be surprising - the naive (non-robust) fit should be measurably
    # worse than the noiseless case, proving the outliers actually matter
    # to this test (otherwise --robust would have nothing to demonstrate).
    assert np.linalg.norm(result.bias - dataset.true_bias) > 0.5 or abs(result.reference_magnitude - dataset.true_reference_magnitude) > 0.5


def test_robust_rejection_recovers_close_to_ground_truth_despite_outliers():
    dataset = synthetic.with_outliers()
    kept_points, mask, result = reject_outliers(dataset.points)
    assert mask.sum() < dataset.points.shape[0]  # actually dropped something
    np.testing.assert_allclose(result.bias, dataset.true_bias, atol=1.0)
    assert abs(result.reference_magnitude - dataset.true_reference_magnitude) < 1.0


def test_too_few_points_raises():
    with pytest.raises(EllipsoidFitError):
        fit_ellipsoid(np.array([[1.0, 0, 0], [0, 1, 0], [0, 0, 1]]))


def test_insufficient_coverage_raises_or_is_degenerate():
    dataset = synthetic.insufficient_coverage()
    # A narrow cone of directions under-constrains the general ellipsoid
    # fit - either it raises outright (rank-deficient/singular), or it
    # "succeeds" with residuals far too large to trust; either outcome is
    # acceptable here (quality.py is what turns this into a hard stop for
    # the CLI), this test just documents the fit layer doesn't fabricate a
    # clean-looking answer from bad geometry.
    try:
        result = fit_ellipsoid(dataset.points)
    except EllipsoidFitError:
        return
    rms_residual = float(np.sqrt(np.mean(result.residuals ** 2)))
    assert rms_residual > 0.5 or not np.isfinite(result.reference_magnitude)


def test_flat_only_rotation_raises_singular():
    dataset = synthetic.flat_only_rotation()
    with pytest.raises(EllipsoidFitError):
        fit_ellipsoid(dataset.points)


def test_duplicate_static_raises_or_degenerate():
    dataset = synthetic.duplicate_static()
    with pytest.raises(EllipsoidFitError):
        fit_ellipsoid(dataset.points)


def test_wrong_shape_input_raises():
    with pytest.raises(EllipsoidFitError):
        fit_ellipsoid(np.zeros((20, 2)))
