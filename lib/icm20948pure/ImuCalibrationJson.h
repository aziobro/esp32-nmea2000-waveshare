#pragma once
#include "ImuTypes.h"
#include "ImuCalibration.h"
#include "ImuDeviationTable.h"
#include <ArduinoJson.h>
#include <string>

/*
  Versioned JSON import/export for the complete calibration state
  (magnetometer + gyro calibration, mounting orientation, fixed heading
  offset, deviation table). Uses ArduinoJson (already this project's
  established JSON library everywhere else - pure C++, no Arduino.h
  dependency despite the name, so this is testable on desktop).

  Import is all-or-nothing: importJson() only ever mutates its output
  parameters after every check has passed. A malformed or implausible
  file leaves the caller's existing calibration completely untouched and
  returns a specific, human-readable reason via outError - never a
  partial write, and the caller is expected to not persist anything to
  NVS unless importJson() returns true (see doc/IcmHeadingArchitecture.md).
*/
namespace ImuCalibrationJson
{
    // Current schema version this code writes and accepts on import.
    static const int SCHEMA_VERSION = 1;

    std::string exportJson(const ImuCalibration &cal, MountOrientation orientation,
                       const DeviationTable &deviationTable, bool deviationEnabled);

    // Returns true and fills every output parameter only if the entire
    // document validates cleanly (schema version, numeric ranges,
    // finite values, matrix plausibility, array shapes). Returns false
    // and leaves all output parameters untouched otherwise, with
    // outError describing exactly what failed.
    bool importJson(const std::string &json, ImuCalibration &outCal, MountOrientation &outOrientation,
                     DeviationTable &outDeviationTable, bool &outDeviationEnabled, std::string &outError);

    // Identity/safe-default calibration as JSON, for a "reset to
    // identity" web control - same content exportJson() would produce
    // for ImuCalibrationOps::identityDefault() at MountOrientation::Forward
    // with an empty, disabled deviation table.
    std::string identityJson();
}
