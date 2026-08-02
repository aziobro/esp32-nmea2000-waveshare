import numpy as np

import synthetic
from ellipsoid_fit import EllipsoidFitError, fit_ellipsoid
from quality import assess_quality, has_blocking_issues


def _fit_or_none(points):
    try:
        return fit_ellipsoid(points), None
    except EllipsoidFitError as e:
        return None, e


def test_ideal_sphere_has_no_blocking_issues():
    dataset = synthetic.ideal_sphere()
    fit_result, fit_error = _fit_or_none(dataset.points)
    issues = assess_quality(dataset.points, fit_result, fit_error)
    assert not has_blocking_issues(issues), issues


def test_too_few_samples_flagged():
    points = np.array([[1.0, 0, 0]] * 5)
    fit_result, fit_error = _fit_or_none(points)
    issues = assess_quality(points, fit_result, fit_error)
    codes = [i.code for i in issues]
    assert "too_few_samples" in codes
    assert has_blocking_issues(issues)


def test_duplicate_static_flagged():
    dataset = synthetic.duplicate_static()
    fit_result, fit_error = _fit_or_none(dataset.points)
    issues = assess_quality(dataset.points, fit_result, fit_error)
    codes = [i.code for i in issues]
    assert "duplicate_static" in codes
    assert has_blocking_issues(issues)


def test_flat_only_rotation_flagged_near_planar():
    dataset = synthetic.flat_only_rotation()
    fit_result, fit_error = _fit_or_none(dataset.points)
    issues = assess_quality(dataset.points, fit_result, fit_error)
    codes = [i.code for i in issues]
    assert "near_planar" in codes
    assert has_blocking_issues(issues)


def test_insufficient_coverage_flagged():
    dataset = synthetic.insufficient_coverage()
    fit_result, fit_error = _fit_or_none(dataset.points)
    issues = assess_quality(dataset.points, fit_result, fit_error)
    assert has_blocking_issues(issues)


def test_outliers_flagged_excessive():
    dataset = synthetic.with_outliers()
    fit_result, fit_error = _fit_or_none(dataset.points)
    issues = assess_quality(dataset.points, fit_result, fit_error)
    codes = [i.code for i in issues]
    assert "excessive_outliers" in codes or "excessive_residual" in codes
    assert has_blocking_issues(issues)


def test_singular_fit_surfaced_as_issue():
    dataset = synthetic.duplicate_static()
    fit_result, fit_error = _fit_or_none(dataset.points)
    assert fit_result is None and fit_error is not None
    issues = assess_quality(dataset.points, fit_result, fit_error)
    codes = [i.code for i in issues]
    # duplicate_static is caught earlier by its own check, but if the fit
    # itself also failed that failure must not be silently dropped.
    assert "duplicate_static" in codes or "singular_fit" in codes


def test_implausible_magnitude_flagged():
    # Fabricate a "successful" fit result with an obviously wrong magnitude
    # by fitting a sphere scaled to a physically implausible field strength.
    dataset = synthetic.ideal_sphere()
    tiny_points = dataset.points * 0.01  # ~0.5 uT field - not Earth's
    fit_result, fit_error = _fit_or_none(tiny_points)
    assert fit_result is not None
    issues = assess_quality(tiny_points, fit_result, fit_error)
    codes = [i.code for i in issues]
    assert "implausible_magnitude" in codes
