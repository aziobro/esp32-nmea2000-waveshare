#pragma once

/*
  Circular-angle utilities, all in degrees unless noted. Ordinary
  subtraction/averaging breaks near the 0/360 wrap (e.g. averaging 359 and 1
  with plain arithmetic gives 180, not 0) - every function here is careful
  about that instead.
*/
namespace ImuAngleMath
{
    // [0, 360)
    double wrap360(double deg);
    // (-180, 180]
    double wrap180(double deg);

    // Shortest signed path from `fromDeg` to `toDeg`, range (-180, 180].
    // Positive = toDeg is clockwise (to starboard) of fromDeg.
    double shortestDiff(double fromDeg, double toDeg);

    // Interpolate along the shortest circular path from a to b. t=0 -> a,
    // t=1 -> b (not clamped - callers wanting a hold at the ends should
    // clamp before calling).
    double circularInterpolate(double aDeg, double bDeg, double t);

    // Weighted circular mean via the vector-sum method (sum unit vectors at
    // each angle scaled by weight, take the resultant's angle) - the only
    // correct way to average angles, a weighted arithmetic mean is wrong
    // near the wrap. Returns 0 if n<=0 or all weights are zero.
    double circularWeightedAverage(const double *anglesDeg, const double *weights, int n);

    // One step of a circular exponential low-pass filter: moves `prevDeg`
    // toward `newDeg` by fraction `alpha` (0=no movement, 1=snap to
    // newDeg), always via the shortest circular path.
    double circularLowPass(double prevDeg, double newDeg, double alpha);

    // Accumulates a continuously-updated angle into an unwrapped (not
    // wrapped to 0-360) running total, so ordinary arithmetic (rate of
    // change, comparison) works on the result without special-casing the
    // wrap. Each update() call must be reasonably frequent relative to the
    // angle's rate of change - a jump of more than 180 degrees between
    // calls is inherently ambiguous (can't tell a big clockwise turn from a
    // big counter-clockwise one) and is resolved as the shorter one.
    class UnwrappedAccumulator
    {
        double lastDeg = 0;
        double unwrapped = 0;
        bool initialized = false;

    public:
        void reset(double deg);
        // Feeds a new wrapped (0-360 or any range) sample, returns the new
        // unwrapped total.
        double update(double deg);
        double value() const { return unwrapped; }
        bool isInitialized() const { return initialized; }
    };
}
