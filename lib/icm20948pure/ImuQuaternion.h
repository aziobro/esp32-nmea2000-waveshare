#pragma once
#include "ImuTypes.h"

/*
  Quaternion utilities: normalization, validity checks, and conversion to/
  from Euler angles (roll/pitch/yaw, radians, boat-frame sign convention -
  positive roll=starboard heel, positive pitch=bow up). The ZYX (yaw then
  pitch then roll) Tait-Bryan extraction formula matches the pre-rewrite
  code's quatToEuler exactly (moved here unchanged); fromEuler is its
  matched inverse (standard ZYX quaternion-from-Euler formula), added so
  the simulator and tests can generate known-attitude quaternions.
*/
namespace ImuQuaternion
{
    double normSquared(const Quaternion &q);
    double norm(const Quaternion &q);

    // Returns a unit quaternion. If the input is degenerate (near-zero
    // norm), returns the identity quaternion rather than dividing by
    // (near) zero.
    Quaternion normalize(const Quaternion &q);

    // True if w/x/y/z are all finite (not NaN/Inf) - the first check any
    // DMP sample should go through before anything else touches it.
    bool isFinite(const Quaternion &q);

    // True if isFinite AND the norm is within tolerance of 1.0 - a unit
    // quaternion is a valid rotation, one that isn't (badly out of range)
    // usually means corrupted FIFO data.
    bool isValidUnit(const Quaternion &q, double tolerance = 0.05);

    void toEuler(const Quaternion &q, double &rollRad, double &pitchRad, double &yawRad);
    Quaternion fromEuler(double rollRad, double pitchRad, double yawRad);

    // Shortest angular difference between two quaternions' yaw components
    // alone (degrees) - used to compare DMP heading against the software
    // compass/fusion heading without needing a full quaternion distance
    // metric.
    double yawDifferenceDeg(const Quaternion &a, const Quaternion &b);
}
