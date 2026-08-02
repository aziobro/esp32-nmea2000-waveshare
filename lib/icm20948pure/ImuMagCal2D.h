#pragma once
#include "ImuTypes.h"

/*
  Live, on-device "boat swing" magnetometer calibration - a pure state
  machine (no hardware I/O, no NVS access; the task layer feeds it raw
  boat-frame magnetometer samples and reads results back, same pattern as
  GyroCalEngine in ImuGyroCal.h).

  This is deliberately a 2D (X/Y only) hard-iron-plus-axis-scale fit, not
  a full 3D ellipsoid: a boat can only be swung through headings (yaw),
  never tumbled through arbitrary 3D orientations the way a handheld
  sensor can be on a workbench - see ImuCalibrationOps::migrateFromLegacy's
  comment for the same constraint on the pre-rewrite code. Z bias/scale is
  left untouched (identity) here on purpose. The full 3D ellipsoid fit
  (tools/icm20948_calibration/ellipsoid_fit.py) is the "real" calibration
  path for a sensor removed from the boat and tumbled by hand; this engine
  exists for a quick, no-tools-needed, standing-on-the-boat calibration
  that's better than nothing and matches how a magnetic compass is
  traditionally "swung."

  Uses Kasa's algebraic circle fit (x^2+y^2 = D*x + E*y + F, linear in
  D/E/F - a standard, simple closed-form circle fit) for the hard-iron
  center, plus a mean-squared-radius comparison for a bounded-memory
  residual/quality proxy. Deliberately accumulates only a fixed set of
  running sums (never stores per-sample data) so calibration duration
  never affects memory use - see doc/IcmImplementationAudit.md's Phase 7
  concerns about unbounded containers in this codebase's real-time paths.
*/

enum class MagCal2DState
{
    Idle,
    Collecting,
    Ready,
    Failed
};

struct MagCal2DConfig
{
    // All provisional - see doc/IcmHeadingArchitecture.md; real values
    // need tuning against an actual boat swing.
    int sectorCount = 16;               // clamped to [1, ImuMagCal2D::MAX_SECTORS]
    int minSamples = 200;
    double minCoverageFraction = 0.7;   // fraction of sectors that must be visited
    double minFieldMagnitudeUT = 5.0;   // per-sample plausibility gate
    double maxFieldMagnitudeUT = 150.0;
    double maxResidualFraction = 0.25;  // |meanDist^2 - r^2| / r^2 limit
};

class ImuMagCal2D
{
public:
    static const int MAX_SECTORS = 36;

    void start();
    void cancel();

    // Feeds one raw boat-frame magnetometer sample's X/Y components.
    // Returns false if the sample was rejected (implausible field
    // magnitude) - rejected samples don't count toward progress but don't
    // reset it either (a transient bad reading shouldn't throw away an
    // otherwise-good swing in progress).
    bool addSample(double magBoatX, double magBoatY, const MagCal2DConfig &cfg = MagCal2DConfig());

    // Stops collecting and attempts the fit now. Transitions to Ready
    // (fit accepted) or Failed (insufficient samples/coverage, or a
    // numerically degenerate fit) - either way, sampling stops. No-op
    // (returns false, state unchanged) if not currently Collecting.
    bool stop(const MagCal2DConfig &cfg = MagCal2DConfig());

    MagCal2DState state() const { return st; }
    int sampleCount() const { return count; }

    // Fraction of sectors visited at least once (0..1). Meaningful in any
    // state once sampling has started.
    double coverageFraction(const MagCal2DConfig &cfg = MagCal2DConfig()) const;

    // 360 degrees minus the largest contiguous unvisited arc - distinct
    // from coverageFraction() when sectors are touched non-contiguously
    // (e.g. wandering back and forth over the same arc still scores high
    // on coverageFraction's sector count but low here).
    double spanDeg(const MagCal2DConfig &cfg = MagCal2DConfig()) const;

    // Empty ("") unless state()==Failed.
    const char *failureReason() const { return failReason; }

    // Valid only once state()==Ready. Z bias/scale are intentionally not
    // part of this engine's output - see class comment.
    double resultBiasX() const { return biasX; }
    double resultBiasY() const { return biasY; }
    double resultScaleX() const { return scaleX; }
    double resultScaleY() const { return scaleY; }
    double resultQuality() const { return quality; }

private:
    MagCal2DState st = MagCal2DState::Idle;
    int count = 0;

    double sumX = 0, sumY = 0, sumXX = 0, sumYY = 0, sumXY = 0;
    double sumXR2 = 0, sumYR2 = 0, sumR2 = 0; // R2 = x^2+y^2, the Kasa fit's linear target

    bool sectorVisited[MAX_SECTORS] = {};

    double biasX = 0, biasY = 0, scaleX = 1, scaleY = 1, quality = 0;
    const char *failReason = "";

    void fail(const char *reason);
};
