#include "ImuCalibrationJson.h"
#include <math.h>
#include <stdio.h>

namespace
{
    bool finiteAndInRange(double v, double lo, double hi)
    {
        return isfinite(v) && v >= lo && v <= hi;
    }
}

namespace ImuCalibrationJson
{
    std::string exportJson(const ImuCalibration &cal, MountOrientation orientation,
                       const DeviationTable &deviationTable, bool deviationEnabled)
    {
        DynamicJsonDocument doc(8192);
        doc["schemaVersion"] = SCHEMA_VERSION;
        doc["orientation"] = static_cast<int>(orientation);

        JsonObject mag = doc.createNestedObject("magCalibration");
        mag["valid"] = cal.magCalibrationValid;
        mag["quality"] = cal.magCalibrationQuality;
        mag["referenceMagnitude"] = 0.0; // not yet tracked by ImuCalibration - reserved for the 2D/3D cal engines
        JsonArray bias = mag.createNestedArray("bias");
        bias.add(cal.magBias[0]);
        bias.add(cal.magBias[1]);
        bias.add(cal.magBias[2]);
        JsonArray matrix = mag.createNestedArray("matrix");
        for (int i = 0; i < 3; i++)
        {
            JsonArray row = matrix.createNestedArray();
            row.add(cal.magMatrix[i][0]);
            row.add(cal.magMatrix[i][1]);
            row.add(cal.magMatrix[i][2]);
        }

        JsonObject gyro = doc.createNestedObject("gyroCalibration");
        gyro["valid"] = cal.gyroCalibrationValid;
        JsonArray gbias = gyro.createNestedArray("bias");
        gbias.add(cal.gyroBias[0]);
        gbias.add(cal.gyroBias[1]);
        gbias.add(cal.gyroBias[2]);
        JsonArray gstd = gyro.createNestedArray("standardDeviation");
        gstd.add(0.0);
        gstd.add(0.0);
        gstd.add(0.0);

        JsonObject heading = doc.createNestedObject("heading");
        heading["fixedOffsetDeg"] = cal.fixedHeadingOffsetDeg;

        JsonObject dev = doc.createNestedObject("deviationTable");
        dev["enabled"] = deviationEnabled;
        JsonArray entries = dev.createNestedArray("entries");
        for (int i = 0; i < deviationTable.size(); i++)
        {
            DeviationEntry e = deviationTable.entryAt(i);
            JsonObject entry = entries.createNestedObject();
            entry["measuredHeadingDeg"] = e.measuredHeadingDeg;
            entry["correctionDeg"] = e.correctionDeg;
        }

        std::string out;
        serializeJson(doc, out);
        return out;
    }

    std::string identityJson()
    {
        ImuCalibration cal = ImuCalibrationOps::identityDefault();
        DeviationTable empty;
        return exportJson(cal, MountOrientation::Forward, empty, false);
    }

    bool importJson(const std::string &json, ImuCalibration &outCal, MountOrientation &outOrientation,
                     DeviationTable &outDeviationTable, bool &outDeviationEnabled, std::string &outError)
    {
        // Sized with headroom beyond DeviationTable::MAX_ENTRIES so an
        // over-limit document still parses successfully and hits the
        // explicit entries.size() check below with a specific error,
        // rather than failing generically here with NoMemory.
        DynamicJsonDocument doc(8192);
        DeserializationError err = deserializeJson(doc, json);
        if (err)
        {
            outError = std::string("malformed JSON: ") + err.c_str();
            return false;
        }

        if (!doc.containsKey("schemaVersion") || !doc["schemaVersion"].is<int>())
        {
            outError = "missing or non-numeric schemaVersion";
            return false;
        }
        int schemaVersion = doc["schemaVersion"].as<int>();
        if (schemaVersion != SCHEMA_VERSION)
        {
            outError = "unsupported schemaVersion " + std::to_string(schemaVersion) + " (this firmware accepts " + std::to_string(SCHEMA_VERSION) + ")";
            return false;
        }

        if (!doc.containsKey("orientation") || !doc["orientation"].is<int>())
        {
            outError = "missing or non-numeric orientation";
            return false;
        }
        int orientationIdx = doc["orientation"].as<int>();
        if (orientationIdx < 0 || orientationIdx > 23)
        {
            outError = "orientation out of range (must be 0-23), got " + std::to_string(orientationIdx);
            return false;
        }

        if (!doc.containsKey("magCalibration") || !doc["magCalibration"].is<JsonObject>())
        {
            outError = "missing magCalibration object";
            return false;
        }
        JsonObject mag = doc["magCalibration"];

        if (!mag.containsKey("bias") || !mag["bias"].is<JsonArray>() || mag["bias"].as<JsonArray>().size() != 3)
        {
            outError = "magCalibration.bias must be an array of exactly 3 numbers";
            return false;
        }
        double magBias[3];
        {
            JsonArray a = mag["bias"];
            for (int i = 0; i < 3; i++)
            {
                if (!a[i].is<float>() && !a[i].is<int>())
                {
                    outError = "magCalibration.bias[" + std::to_string(i) + "] is not numeric";
                    return false;
                }
                magBias[i] = a[i].as<double>();
                if (!finiteAndInRange(magBias[i], -2000.0, 2000.0))
                {
                    outError = "magCalibration.bias[" + std::to_string(i) + "] is not finite or is out of plausible range (+/-2000)";
                    return false;
                }
            }
        }

        if (!mag.containsKey("matrix") || !mag["matrix"].is<JsonArray>() || mag["matrix"].as<JsonArray>().size() != 3)
        {
            outError = "magCalibration.matrix must be an array of exactly 3 rows";
            return false;
        }
        double magMatrix[3][3];
        {
            JsonArray rows = mag["matrix"];
            for (int i = 0; i < 3; i++)
            {
                if (!rows[i].is<JsonArray>() || rows[i].as<JsonArray>().size() != 3)
                {
                    outError = "magCalibration.matrix row " + std::to_string(i) + " must have exactly 3 numbers";
                    return false;
                }
                JsonArray row = rows[i];
                for (int j = 0; j < 3; j++)
                {
                    if (!row[j].is<float>() && !row[j].is<int>())
                    {
                        outError = "magCalibration.matrix[" + std::to_string(i) + "][" + std::to_string(j) + "] is not numeric";
                        return false;
                    }
                    magMatrix[i][j] = row[j].as<double>();
                    if (!finiteAndInRange(magMatrix[i][j], -10.0, 10.0))
                    {
                        outError = "magCalibration.matrix[" + std::to_string(i) + "][" + std::to_string(j) + "] is not finite or is out of plausible range (+/-10)";
                        return false;
                    }
                }
            }
        }
        // Reject a singular or implausibly-scaled matrix - a real
        // soft-iron correction should have a determinant reasonably
        // close to 1 (moderate scale correction), never near 0
        // (singular - not invertible, couldn't have come from a real
        // fit) and never huge (would imply a nonsensical multi-hundred-
        // percent correction).
        double det = magMatrix[0][0] * (magMatrix[1][1] * magMatrix[2][2] - magMatrix[1][2] * magMatrix[2][1]) -
                     magMatrix[0][1] * (magMatrix[1][0] * magMatrix[2][2] - magMatrix[1][2] * magMatrix[2][0]) +
                     magMatrix[0][2] * (magMatrix[1][0] * magMatrix[2][1] - magMatrix[1][1] * magMatrix[2][0]);
        if (!isfinite(det) || fabs(det) < 0.01 || fabs(det) > 50.0)
        {
            char detStr[32];
            snprintf(detStr, sizeof(detStr), "%.4f", det);
            outError = "magCalibration.matrix is singular or implausibly scaled (determinant " + std::string(detStr) + ")";
            return false;
        }

        double magQuality = 0;
        if (mag.containsKey("quality"))
        {
            if (!mag["quality"].is<float>() && !mag["quality"].is<int>())
            {
                outError = "magCalibration.quality is not numeric";
                return false;
            }
            magQuality = mag["quality"].as<double>();
            if (!finiteAndInRange(magQuality, 0.0, 1.0))
            {
                outError = "magCalibration.quality must be between 0 and 1";
                return false;
            }
        }
        bool magValid = mag.containsKey("valid") ? mag["valid"].as<bool>() : true;

        // --- gyroCalibration (optional block - a file exported before
        // gyro calibration existed, or hand-written, may omit it
        // entirely; defaults to zero/invalid, never a validation
        // failure on its own). ---
        double gyroBias[3] = {0, 0, 0};
        bool gyroValid = false;
        if (doc.containsKey("gyroCalibration"))
        {
            if (!doc["gyroCalibration"].is<JsonObject>())
            {
                outError = "gyroCalibration must be an object";
                return false;
            }
            JsonObject gyro = doc["gyroCalibration"];
            if (gyro.containsKey("bias"))
            {
                if (!gyro["bias"].is<JsonArray>() || gyro["bias"].as<JsonArray>().size() != 3)
                {
                    outError = "gyroCalibration.bias must be an array of exactly 3 numbers";
                    return false;
                }
                JsonArray a = gyro["bias"];
                for (int i = 0; i < 3; i++)
                {
                    if (!a[i].is<float>() && !a[i].is<int>())
                    {
                        outError = "gyroCalibration.bias[" + std::to_string(i) + "] is not numeric";
                        return false;
                    }
                    gyroBias[i] = a[i].as<double>();
                    if (!finiteAndInRange(gyroBias[i], -100.0, 100.0))
                    {
                        outError = "gyroCalibration.bias[" + std::to_string(i) + "] is not finite or is out of plausible range (+/-100 deg/s)";
                        return false;
                    }
                }
            }
            gyroValid = gyro.containsKey("valid") ? gyro["valid"].as<bool>() : false;
        }

        // --- heading.fixedOffsetDeg ---
        double fixedOffsetDeg = 0;
        if (doc.containsKey("heading"))
        {
            if (!doc["heading"].is<JsonObject>())
            {
                outError = "heading must be an object";
                return false;
            }
            JsonObject heading = doc["heading"];
            if (heading.containsKey("fixedOffsetDeg"))
            {
                if (!heading["fixedOffsetDeg"].is<float>() && !heading["fixedOffsetDeg"].is<int>())
                {
                    outError = "heading.fixedOffsetDeg is not numeric";
                    return false;
                }
                fixedOffsetDeg = heading["fixedOffsetDeg"].as<double>();
                if (!finiteAndInRange(fixedOffsetDeg, -360.0, 360.0))
                {
                    outError = "heading.fixedOffsetDeg is not finite or is out of range (+/-360)";
                    return false;
                }
            }
        }

        // --- deviationTable (optional) ---
        bool deviationEnabled = false;
        DeviationTable deviationTable;
        if (doc.containsKey("deviationTable"))
        {
            if (!doc["deviationTable"].is<JsonObject>())
            {
                outError = "deviationTable must be an object";
                return false;
            }
            JsonObject dev = doc["deviationTable"];
            deviationEnabled = dev.containsKey("enabled") ? dev["enabled"].as<bool>() : false;
            if (dev.containsKey("entries"))
            {
                if (!dev["entries"].is<JsonArray>())
                {
                    outError = "deviationTable.entries must be an array";
                    return false;
                }
                JsonArray entries = dev["entries"];
                if ((int)entries.size() > DeviationTable::MAX_ENTRIES)
                {
                    outError = "deviationTable.entries has more than " + std::to_string(DeviationTable::MAX_ENTRIES) + " entries";
                    return false;
                }
                int idx = 0;
                for (JsonObject entry : entries)
                {
                    if (!entry.containsKey("measuredHeadingDeg") || !entry.containsKey("correctionDeg"))
                    {
                        outError = "deviationTable.entries[" + std::to_string(idx) + "] missing measuredHeadingDeg or correctionDeg";
                        return false;
                    }
                    double measured = entry["measuredHeadingDeg"].as<double>();
                    double correction = entry["correctionDeg"].as<double>();
                    if (!finiteAndInRange(measured, 0.0, 360.0))
                    {
                        outError = "deviationTable.entries[" + std::to_string(idx) + "].measuredHeadingDeg is not finite or out of range (0-360)";
                        return false;
                    }
                    if (!finiteAndInRange(correction, -90.0, 90.0))
                    {
                        outError = "deviationTable.entries[" + std::to_string(idx) + "].correctionDeg is not finite or is out of plausible range (+/-90)";
                        return false;
                    }
                    if (!deviationTable.addEntry(measured, measured + correction))
                    {
                        outError = "deviationTable.entries[" + std::to_string(idx) + "] could not be added (table full)";
                        return false;
                    }
                    idx++;
                }
            }
        }

        // --- Every check passed - commit to the output parameters. ---
        outCal.magBias[0] = magBias[0];
        outCal.magBias[1] = magBias[1];
        outCal.magBias[2] = magBias[2];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                outCal.magMatrix[i][j] = magMatrix[i][j];
        outCal.magCalibrationValid = magValid;
        outCal.magCalibrationQuality = magQuality;
        outCal.gyroBias[0] = gyroBias[0];
        outCal.gyroBias[1] = gyroBias[1];
        outCal.gyroBias[2] = gyroBias[2];
        outCal.gyroCalibrationValid = gyroValid;
        outCal.fixedHeadingOffsetDeg = fixedOffsetDeg;
        outCal.calibrationSequence++;

        outOrientation = static_cast<MountOrientation>(orientationIdx);
        outDeviationTable = deviationTable;
        outDeviationEnabled = deviationEnabled;
        outError = "";
        return true;
    }
}
