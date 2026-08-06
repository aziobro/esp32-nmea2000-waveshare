#include "ImuDiagnostics.h"
#include <stdio.h>

namespace ImuDiagnostics
{
    const char *csvHeader()
    {
        return "timestamp_ms,sample_sequence,"
               "accel_raw_x,accel_raw_y,accel_raw_z,"
               "gyro_raw_x,gyro_raw_y,gyro_raw_z,"
               "mag_raw_x,mag_raw_y,mag_raw_z,"
               "accel_boat_x,accel_boat_y,accel_boat_z,"
               "gyro_boat_x,gyro_boat_y,gyro_boat_z,"
               "mag_boat_x,mag_boat_y,mag_boat_z,"
               "mag_corrected_x,mag_corrected_y,mag_corrected_z,"
               "mag_magnitude,"
               "dmp_q0,dmp_q1,dmp_q2,dmp_q3,"
               "dmp_roll_deg,dmp_pitch_deg,dmp_heading_deg,"
               "compass_heading_deg,fusion_heading_deg,output_heading_deg,"
               "rate_of_turn_deg_s,"
               "active_heading_source,heading_quality,rejection_flags,"
               "dmp_sample_age_ms,dmp_compass_age_ms,"
               "fifo_error_count,fifo_drain_limit_count,sensor_error_count";
    }

    const char *headingSourceCsvName(HeadingSource s)
    {
        switch (s)
        {
        case HeadingSource::Dmp:
            return "dmp";
        case HeadingSource::SoftwareCompass:
            return "software_compass";
        case HeadingSource::SoftwareFusion:
            return "software_9axis_fusion";
        default:
            return "none";
        }
    }

    const char *headingQualityCsvName(HeadingQuality q)
    {
        switch (q)
        {
        case HeadingQuality::Good:
            return "good";
        case HeadingQuality::Poor:
            return "poor";
        default:
            return "invalid";
        }
    }

    int formatCsvRow(const DiagnosticSample &s, char *out, size_t outCapacity)
    {
        int n = snprintf(out, outCapacity,
                          "%lu,%lu,"
                          "%.4f,%.4f,%.4f,"
                          "%.4f,%.4f,%.4f,"
                          "%.2f,%.2f,%.2f,"
                          "%.4f,%.4f,%.4f,"
                          "%.4f,%.4f,%.4f,"
                          "%.2f,%.2f,%.2f,"
                          "%.2f,%.2f,%.2f,"
                          "%.2f,"
                          "%.5f,%.5f,%.5f,%.5f,"
                          "%.2f,%.2f,%.2f,"
                          "%.2f,%.2f,%.2f,"
                          "%.2f,"
                          "%s,%s,%lu,"
                          "%d,%d,%lu,%lu,%lu",
                          (unsigned long)s.timestampMs, (unsigned long)s.sampleSequence,
                          s.accelRaw.x, s.accelRaw.y, s.accelRaw.z,
                          s.gyroRaw.x, s.gyroRaw.y, s.gyroRaw.z,
                          s.magRaw.x, s.magRaw.y, s.magRaw.z,
                          s.accelBoat.x, s.accelBoat.y, s.accelBoat.z,
                          s.gyroBoat.x, s.gyroBoat.y, s.gyroBoat.z,
                          s.magBoat.x, s.magBoat.y, s.magBoat.z,
                          s.magCorrected.x, s.magCorrected.y, s.magCorrected.z,
                          s.magMagnitude,
                          s.dmpQ0, s.dmpQ1, s.dmpQ2, s.dmpQ3,
                          s.dmpRollDeg, s.dmpPitchDeg, s.dmpHeadingDeg,
                          s.compassHeadingDeg, s.fusionHeadingDeg, s.outputHeadingDeg,
                          s.rateOfTurnDegPerSec,
                          headingSourceCsvName(s.activeSource), headingQualityCsvName(s.headingQuality), (unsigned long)s.rejectionFlags,
                          s.dmpSampleAgeMs, s.dmpCompassAgeMs,
                          (unsigned long)s.fifoErrorCount,
                          (unsigned long)s.fifoDrainLimitCount,
                          (unsigned long)s.sensorErrorCount);
        if (n < 0 || (size_t)n >= outCapacity)
            return -1;
        return n;
    }
}
