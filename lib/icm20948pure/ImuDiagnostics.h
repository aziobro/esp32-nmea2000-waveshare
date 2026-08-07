#pragma once
#include "ImuTypes.h"
#include <stdint.h>
#include <stddef.h>

/*
  One full diagnostic sample (every stage of the pipeline, one struct per
  cycle) and pure CSV formatting for it - no dynamic allocation, no
  Arduino/String dependency, so it's usable identically from the
  high-rate IMU loop and from host-side tests. The runtime capture
  system that queues/writes these (ring buffer, serial/HTTP output,
  start/stop controls) lives in lib/icm20948task (Arduino-dependent) -
  see GwIcm20948CaptureTask.h.
*/

struct DiagnosticSample
{
    uint32_t timestampMs = 0;
    uint32_t sampleSequence = 0;

    Vec3 accelRaw, gyroRaw, magRaw;
    Vec3 accelBoat, gyroBoat, magBoat;
    Vec3 magCorrected;
    double magMagnitude = 0;

    double compassHeadingDeg = 0;
    double fusionHeadingDeg = 0;
    double outputHeadingDeg = 0;
    double rateOfTurnDegPerSec = 0;

    HeadingSource activeSource = HeadingSource::None;
    HeadingQuality headingQuality = HeadingQuality::Invalid;
    uint32_t rejectionFlags = 0;

    uint32_t sensorErrorCount = 0;
};

namespace ImuDiagnostics
{
    // Column order matches this exact list (see the project's CSV
    // capture spec): timestamp_ms,sample_sequence,accel_raw_x..z,
    // gyro_raw_x..z,mag_raw_x..z,accel_boat_x..z,gyro_boat_x..z,
    // mag_boat_x..z,mag_corrected_x..z,mag_magnitude,compass_heading_deg,
    // fusion_heading_deg,output_heading_deg,rate_of_turn_deg_s,
    // active_heading_source,heading_quality,rejection_flags,sensor_error_count
    const char *csvHeader();

    // Formats one CSV row (no trailing newline) into a caller-supplied
    // buffer - no allocation. Returns the number of characters written
    // (excluding the null terminator), or a negative value if
    // outCapacity was too small to hold the full row.
    int formatCsvRow(const DiagnosticSample &s, char *out, size_t outCapacity);

    const char *headingSourceCsvName(HeadingSource s);
    const char *headingQualityCsvName(HeadingQuality q);
}
