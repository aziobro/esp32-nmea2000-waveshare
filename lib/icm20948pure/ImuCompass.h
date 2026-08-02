#pragma once
#include "ImuTypes.h"

/*
  Tilt-compensated magnetic compass: projects a magnetometer reading onto
  the boat's horizontal plane using the current roll/pitch, then takes the
  heading from the projected components. Formula unchanged from the
  pre-rewrite code (a standard tilt-compensated compass formula, the same
  shape as e.g. Freescale/NXP AN4248) - only its home has moved, into a
  pure, independently testable function.

  Input `mag` must already be in boat frame (X=forward, Y=starboard,
  Z=down - after ImuCoordinateTransform) and calibrated (after
  ImuCalibrationOps::applyMag). roll/pitch are boat attitude in radians,
  same sign convention as the rest of this codebase (positive roll =
  heel to starboard, positive pitch = bow rising).
*/
namespace ImuCompass
{
    // Raw heading in degrees, NOT wrapped to [0,360) or offset-corrected -
    // callers apply ImuAngleMath::wrap360, the fixed mounting offset, and
    // any deviation-table correction on top of this.
    double rawHeadingDeg(const Vec3 &mag, double rollRad, double pitchRad);
}
