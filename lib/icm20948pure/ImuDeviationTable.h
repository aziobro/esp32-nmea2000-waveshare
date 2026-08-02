#pragma once
#include "ImuTypes.h"

/*
  Optional compass-deviation table, independent from magnetometer
  calibration (applied after it - see doc/IcmHeadingArchitecture.md for
  the full correction-order writeup). Each entry pairs a measured
  (post-calibration, post-mounting-offset) heading with a trusted
  reference heading; correctionFor() circularly interpolates between the
  nearest entries on either side, wrapping correctly across 359/0.

  Fixed-capacity array (36 entries - the finest granularity the spec
  calls for, 10-degree spacing) rather than a dynamically-sized
  container: avoids heap fragmentation risk on a long-running embedded
  device, and 36 entries is a trivial, bounded amount of memory either
  way.
*/

struct DeviationEntry
{
    double measuredHeadingDeg = 0;
    double correctionDeg = 0; // signed: referenceHeading - measuredHeading, shortest path
};

class DeviationTable
{
public:
    static const int MAX_ENTRIES = 36;

    bool enabled = false;

    void clear();
    int size() const { return count; }
    DeviationEntry entryAt(int index) const;

    // Adds (or, if an entry already exists within 1 degree of
    // measuredHeadingDeg, replaces) a calibration point. Returns false if
    // the table is full and no existing entry is close enough to replace.
    bool addEntry(double measuredHeadingDeg, double referenceHeadingDeg);

    // Circularly interpolated correction (degrees) for a given measured
    // heading - 0 if the table is empty. Not gated by `enabled` (callers
    // check that separately, same as every other optional-correction
    // stage in this pipeline).
    double correctionDegFor(double measuredHeadingDeg) const;

    // measuredHeadingDeg + correctionDegFor(measuredHeadingDeg), wrapped.
    double apply(double measuredHeadingDeg) const;

private:
    DeviationEntry entries[MAX_ENTRIES];
    int count = 0;

    int findNearestIndex(double measuredHeadingDeg) const;
    void insertSorted(const DeviationEntry &e);
};
