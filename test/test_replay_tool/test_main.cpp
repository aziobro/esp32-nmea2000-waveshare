// Debug-only CSV replay tool for the icm20948 heading pipeline.
//
// Runs recorded (or synthetic) sensor samples through the SAME
// ImuCycleProcessor the real hardware task uses (lib/icm20948pure/
// ImuCycleProcessor.h) - not a reimplementation. Builds and runs
// entirely on the desktop; it is not part of any ESP32 board env's
// source tree, so there is no flag or macro that could accidentally
// leave it in a shipped firmware image - this is stronger than a
// compile-time #ifdef, since the code simply isn't reachable from any
// board build at all.
//
// Lives under test/ (not tools/icm20948_replay/, where its README and
// fixtures are) purely for build-system reasons: this project's
// PlatformIO setup can only discover native-buildable code via the
// `pio test` mechanism (per-env `src_dir`/`test_dir` overrides don't
// work here - confirmed by an earlier attempt pulling in the entire
// Arduino-dependent firmware tree instead). It isn't a Unity test and
// asserts nothing; build it with `pio test -e icm20948_native_test
// -f test_replay_tool --without-testing` and run the resulting binary
// directly - see tools/icm20948_replay/README.md for the exact command.
// Run via plain `pio test` (no --without-testing) with no CSV argument,
// it just prints help and exits 0, so it doesn't fail the full native
// suite when that runs without a specific fixture to replay.
#include "ImuCycleProcessor.h"
#include "ImuSimulator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef _WIN32
#include <sys/select.h>
#include <unistd.h>
#endif

namespace
{

const char *EXPECTED_COLUMNS[] = {
    "timestamp_ms", "sample_sequence",
    "accel_raw_x", "accel_raw_y", "accel_raw_z",
    "gyro_raw_x", "gyro_raw_y", "gyro_raw_z",
    "mag_raw_x", "mag_raw_y", "mag_raw_z",
    "accel_boat_x", "accel_boat_y", "accel_boat_z",
    "gyro_boat_x", "gyro_boat_y", "gyro_boat_z",
    "mag_boat_x", "mag_boat_y", "mag_boat_z",
    "mag_corrected_x", "mag_corrected_y", "mag_corrected_z",
    "mag_magnitude",
    "dmp_q0", "dmp_q1", "dmp_q2", "dmp_q3",
    "dmp_roll_deg", "dmp_pitch_deg", "dmp_heading_deg",
    "compass_heading_deg", "fusion_heading_deg", "output_heading_deg",
    "rate_of_turn_deg_s",
    "active_heading_source", "heading_quality", "rejection_flags",
    "dmp_sample_age_ms", "fifo_error_count", "sensor_error_count",
};
const int NUM_COLUMNS = sizeof(EXPECTED_COLUMNS) / sizeof(EXPECTED_COLUMNS[0]);

struct ReplayRow
{
    unsigned long timestampMs = 0;
    Vec3 accelBoat, gyroBoat, magBoat;
    Quaternion dmpQuat; // from dmp_q0..q3
    // Recorded outputs, for --compare ("expected"). outputHeadingDeg < 0
    // means the original capture had no valid heading that cycle.
    double expectedOutputHeadingDeg = -1.0;
    std::string expectedActiveSource;
    std::string expectedHeadingQuality;
};

std::vector<std::string> splitCsvLine(const std::string &line)
{
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ','))
        fields.push_back(field);
    return fields;
}

std::vector<ReplayRow> loadCsv(const std::string &path)
{
    std::ifstream f(path);
    if (!f)
        throw std::runtime_error("cannot open " + path);

    std::string headerLine;
    if (!std::getline(f, headerLine))
        throw std::runtime_error(path + ": empty file");
    std::vector<std::string> header = splitCsvLine(headerLine);
    if ((int)header.size() != NUM_COLUMNS)
        throw std::runtime_error(path + ": expected " + std::to_string(NUM_COLUMNS) + " columns, got " + std::to_string(header.size()));
    for (int i = 0; i < NUM_COLUMNS; i++)
        if (header[i] != EXPECTED_COLUMNS[i])
            throw std::runtime_error(path + ": column " + std::to_string(i) + " is '" + header[i] + "', expected '" + EXPECTED_COLUMNS[i] + "' - not an icm20948 capture CSV");

    std::vector<ReplayRow> rows;
    std::string line;
    int lineNo = 1;
    while (std::getline(f, line))
    {
        lineNo++;
        if (line.empty())
            continue;
        std::vector<std::string> f2 = splitCsvLine(line);
        if ((int)f2.size() != NUM_COLUMNS)
            throw std::runtime_error(path + ":" + std::to_string(lineNo) + ": expected " + std::to_string(NUM_COLUMNS) + " fields, got " + std::to_string(f2.size()));

        ReplayRow row;
        row.timestampMs = std::stoul(f2[0]);
        row.accelBoat = Vec3(std::stod(f2[11]), std::stod(f2[12]), std::stod(f2[13]));
        row.gyroBoat = Vec3(std::stod(f2[14]), std::stod(f2[15]), std::stod(f2[16]));
        row.magBoat = Vec3(std::stod(f2[17]), std::stod(f2[18]), std::stod(f2[19]));
        row.dmpQuat.w = std::stod(f2[24]);
        row.dmpQuat.x = std::stod(f2[25]);
        row.dmpQuat.y = std::stod(f2[26]);
        row.dmpQuat.z = std::stod(f2[27]);
        row.expectedOutputHeadingDeg = std::stod(f2[33]);
        row.expectedActiveSource = f2[35];
        row.expectedHeadingQuality = f2[36];
        rows.push_back(row);
    }
    return rows;
}

// Parses "10", "10:20" (inclusive), or "" (empty range - matches nothing).
struct IndexRange
{
    int from = -1, to = -1;
    bool contains(int i) const { return from >= 0 && i >= from && i <= to; }
};

IndexRange parseRange(const std::string &s)
{
    IndexRange r;
    if (s.empty())
        return r;
    size_t colon = s.find(':');
    if (colon == std::string::npos)
    {
        r.from = r.to = std::stoi(s);
    }
    else
    {
        r.from = std::stoi(s.substr(0, colon));
        r.to = std::stoi(s.substr(colon + 1));
    }
    return r;
}

struct Injections
{
    IndexRange staleDmp;
    IndexRange magDisturbance;
    IndexRange invalidQuaternion;
};

double circularDiffDeg(double a, double b)
{
    double d = std::fmod(std::fabs(a - b), 360.0);
    return d > 180.0 ? 360.0 - d : d;
}

struct CompareStats
{
    int compared = 0;
    int sourceMismatches = 0;
    double maxHeadingDiffDeg = 0;
    double sumHeadingDiffDeg = 0;
};

const char *sourceName(HeadingSource s)
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

void printSample(int index, const ReplayRow &row, const ImuCycleOutput &out)
{
    std::printf("[%4d] t=%6lums  roll=%6.1f pitch=%6.1f  hdg=%s%6.1f  src=%-18s q=%d  rej=0x%03x  rot=%6.1f\n",
                index, row.timestampMs, out.rollDeg, out.pitchDeg,
                out.headingValid ? "" : "-INVALID- ", out.headingValid ? out.headingDeg : 0.0,
                sourceName(out.headingSource), (int)out.headingQuality, out.rejectionFlags, out.rotDegPerSec);
}

void updateCompareStats(CompareStats &stats, const ReplayRow &row, const ImuCycleOutput &out)
{
    if (row.expectedOutputHeadingDeg < 0)
        return; // original capture had no valid heading this cycle - nothing to compare
    if (!out.headingValid)
        return; // replay disagrees about validity itself - not a heading-diff comparison
    stats.compared++;
    double diff = circularDiffDeg(out.headingDeg, row.expectedOutputHeadingDeg);
    stats.sumHeadingDiffDeg += diff;
    if (diff > stats.maxHeadingDiffDeg)
        stats.maxHeadingDiffDeg = diff;
    if (sourceName(out.headingSource) != row.expectedActiveSource)
        stats.sourceMismatches++;
}

void printCompareSummary(const CompareStats &stats)
{
    std::printf("\n--- compare actual (this code) vs expected (recorded in the CSV) ---\n");
    if (stats.compared == 0)
    {
        std::printf("no comparable samples (CSV had no valid recorded heading, or replay produced none)\n");
        return;
    }
    std::printf("compared %d samples: max heading diff %.2f deg, mean %.2f deg, %d/%d active-source mismatches\n",
                stats.compared, stats.maxHeadingDiffDeg, stats.sumHeadingDiffDeg / stats.compared,
                stats.sourceMismatches, stats.compared);
}

ImuCycleInput buildInput(const ReplayRow &row, bool dmpFreshThisCycle, unsigned long dmpAgeMs, unsigned long taskStartMs,
                          double dtSec, bool dmpOk)
{
    ImuCycleInput in;
    in.accelBoat = row.accelBoat;
    in.gyroBoat = row.gyroBoat;
    in.magBoat = row.magBoat;
    in.dmpOk = dmpOk;
    in.haveDmpSample = dmpOk;
    in.dmpFreshThisCycle = dmpFreshThisCycle;
    in.dmpQuat = row.dmpQuat;
    in.dmpAgeMs = dmpAgeMs;
    in.dtSec = dtSec;
    in.nowMs = row.timestampMs;
    in.taskStartMs = taskStartMs;
    // cal/deviationTable/deviationEnabled left at identity/disabled - the
    // CSV's mag_boat_* columns are pre-calibration by construction (see
    // ImuDiagnostics.h), so replaying with identity calibration reproduces
    // the compass/fusion candidates' RAW behavior; pass --reference a real
    // calibration file if you want to replay through a specific one (not
    // yet wired - see README's "known gaps" section).
    in.headingMode = HeadingSourceMode::Auto;
    in.transitionMs = 1000;
    return in;
}

bool stdinLineReady()
{
#ifdef _WIN32
    return false; // interactive pause-during-run isn't supported on Windows builds of this desktop-only tool
#else
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval tv = {0, 0};
    return select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0;
#endif
}

SimulatorState scenarioState(const std::string &name, double t)
{
    using namespace ImuSimulator::Scenarios;
    if (name == "level360Rotation") return level360Rotation(t);
    if (name == "rotation360With20DegHeel") return rotation360With20DegHeel(t);
    if (name == "fixedHeadingChangingAttitude") return fixedHeadingChangingAttitude(t);
    if (name == "hardIronOffset") return hardIronOffset(t);
    if (name == "ellipticalSoftIronDistortion") return ellipticalSoftIronDistortion(t);
    if (name == "suddenMagneticDisturbance") return suddenMagneticDisturbance(t);
    if (name == "slowGyroDrift") return slowGyroDrift(t);
    if (name == "dmpHeadingDisagreement") return dmpHeadingDisagreement(t);
    if (name == "headingWrapThroughNorth") return headingWrapThroughNorth(t);
    if (name == "staleDmpOutput") return staleDmpOutput(t);
    if (name == "quaternionNormError") return quaternionNormError(t);
    if (name == "magnetometerDropout") return magnetometerDropout(t);
    throw std::runtime_error("unknown scenario '" + name + "'");
}

// Generates a fixture CSV by actually running each simulated sample
// through a real ImuCycleProcessor instance (state carried sample to
// sample, exactly like real usage) and recording ITS output as the
// "expected" columns - not an approximation of what the pipeline should
// produce. Replaying this fixture later with --compare should reproduce
// ~0 diff; a nonzero diff means production logic (or this reconstruction
// heuristic) has changed since the fixture was generated - a genuine,
// if coarse, regression signal.
void generateFixture(const std::string &scenario, const std::string &outPath, double durationSec, double rateHz)
{
    std::ofstream f(outPath);
    if (!f)
        throw std::runtime_error("cannot create " + outPath);
    for (int i = 0; i < NUM_COLUMNS; i++)
        f << EXPECTED_COLUMNS[i] << (i + 1 < NUM_COLUMNS ? "," : "\n");

    ImuCycleProcessor processor;
    double dt = 1.0 / rateHz;
    int n = (int)(durationSec / dt);
    unsigned long lastFreshDmpMs = 0;
    unsigned long taskStartMs = 0;

    for (int i = 0; i <= n; i++)
    {
        double t = i * dt;
        SimulatorState state = scenarioState(scenario, t);
        SimulatedSample sample = ImuSimulator::generateSample(state);
        unsigned long tsMs = (unsigned long)(t * 1000.0);

        if (sample.dmpValid)
            lastFreshDmpMs = tsMs;
        unsigned long age = tsMs - lastFreshDmpMs;

        ImuCycleInput in;
        in.accelBoat = sample.accelG;
        in.gyroBoat = sample.gyroDegPerSec;
        in.magBoat = sample.magRaw;
        in.dmpOk = true;
        in.haveDmpSample = true;
        in.dmpFreshThisCycle = sample.dmpValid;
        in.dmpQuat = sample.dmpQuat;
        in.dmpAgeMs = age;
        in.dtSec = (i == 0) ? dt : dt;
        in.nowMs = tsMs;
        in.taskStartMs = taskStartMs;
        in.headingMode = HeadingSourceMode::Auto;
        in.transitionMs = 1000;

        ImuCycleOutput out = processor.process(in);

        f << tsMs << "," << i << ","
          << sample.accelG.x << "," << sample.accelG.y << "," << sample.accelG.z << ","
          << sample.gyroDegPerSec.x << "," << sample.gyroDegPerSec.y << "," << sample.gyroDegPerSec.z << ","
          << sample.magRaw.x << "," << sample.magRaw.y << "," << sample.magRaw.z << ","
          << sample.accelG.x << "," << sample.accelG.y << "," << sample.accelG.z << ","
          << sample.gyroDegPerSec.x << "," << sample.gyroDegPerSec.y << "," << sample.gyroDegPerSec.z << ","
          << sample.magRaw.x << "," << sample.magRaw.y << "," << sample.magRaw.z << ","
          << out.magCorrected.x << "," << out.magCorrected.y << "," << out.magCorrected.z << ","
          << out.magMagnitude << ","
          << sample.dmpQuat.w << "," << sample.dmpQuat.x << "," << sample.dmpQuat.y << "," << sample.dmpQuat.z << ","
          << out.dmpRollDeg << "," << out.dmpPitchDeg << "," << out.rawDmpHeadingDeg << ","
          << out.rawCompassHeadingDeg << "," << out.rawFusionHeadingDeg << "," << (out.headingValid ? out.headingDeg : -1.0) << ","
          << out.rotDegPerSec << ","
          << sourceName(out.headingSource) << "," << (int)out.headingQuality << "," << out.rejectionFlags << ","
          << age << ",0,0\n";
    }
    std::printf("wrote %d samples to %s (scenario=%s, duration=%.1fs, rate=%.1fHz)\n", n + 1, outPath.c_str(), scenario.c_str(), durationSec, rateHz);
}

void printHelp()
{
    std::printf(
        "icm20948_replay - debug-only CSV replay through the real heading pipeline\n\n"
        "usage: icm20948_replay <capture.csv> [options]\n"
        "   or: icm20948_replay --generate <scenario> <output.csv> [durationSec=20] [rateHz=10]\n"
        "       scenarios: level360Rotation, rotation360With20DegHeel, fixedHeadingChangingAttitude,\n"
        "                  hardIronOffset, ellipticalSoftIronDistortion, suddenMagneticDisturbance,\n"
        "                  slowGyroDrift, dmpHeadingDisagreement, headingWrapThroughNorth,\n"
        "                  staleDmpOutput, quaternionNormError, magnetometerDropout\n"
        "  --batch                 run to completion non-interactively and exit\n"
        "  --speed <n>             playback speed multiplier for continuous runs (0 = as fast as possible, default 0)\n"
        "  --loop <n>              repeat the whole file n times (0 = infinite, default 1)\n"
        "  --dmp-ok=true|false     whether DMP is considered active for this run (default true)\n"
        "  --inject-stale-dmp R    force dmp_fresh=false + inflated age for row range R (\"10\" or \"10:20\")\n"
        "  --inject-mag-disturbance R   scale the magnetometer reading for row range R\n"
        "  --inject-invalid-quaternion R  replace the DMP quaternion with a non-unit one for row range R\n"
        "  --compare               print actual-vs-expected diff summary against the CSV's own recorded output\n\n"
        "Interactive commands (default mode, unless --batch):\n"
        "  s [n]      step n samples (default 1)\n"
        "  r          run continuously to completion (or until 'p' or end of loops)\n"
        "  p          pause (checked between samples while running)\n"
        "  speed <n>  change playback speed\n"
        "  loop <n>   change loop count\n"
        "  compare    print the compare summary so far\n"
        "  q          quit\n");
}

} // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        // No CSV given - print help and exit success (not failure), so a
        // full `pio test -e icm20948_native_test` run (which builds and
        // runs every test/test_* binary with no arguments) doesn't treat
        // this debug tool as a failing test.
        printHelp();
        return 0;
    }

    if (std::string(argv[1]) == "--generate")
    {
        if (argc < 4)
        {
            std::fprintf(stderr, "usage: --generate <scenario> <output.csv> [durationSec=20] [rateHz=10]\n");
            return 2;
        }
        try
        {
            double duration = (argc > 4) ? std::stod(argv[4]) : 20.0;
            double rate = (argc > 5) ? std::stod(argv[5]) : 10.0;
            generateFixture(argv[2], argv[3], duration, rate);
        }
        catch (const std::exception &e)
        {
            std::fprintf(stderr, "error: %s\n", e.what());
            return 2;
        }
        return 0;
    }

    std::string csvPath = argv[1];
    bool batch = false;
    int speed = 0;
    int loopCount = 1;
    bool dmpOk = true;
    bool compareRequested = false;
    Injections inj;

    for (int i = 2; i < argc; i++)
    {
        std::string a = argv[i];
        auto val = [&](const std::string &prefix) { return a.substr(prefix.size()); };
        if (a == "--batch")
            batch = true;
        else if (a == "--speed" && i + 1 < argc)
            speed = std::stoi(argv[++i]);
        else if (a == "--loop" && i + 1 < argc)
            loopCount = std::stoi(argv[++i]);
        else if (a.rfind("--dmp-ok=", 0) == 0)
            dmpOk = (val("--dmp-ok=") == "true");
        else if (a == "--inject-stale-dmp" && i + 1 < argc)
            inj.staleDmp = parseRange(argv[++i]);
        else if (a == "--inject-mag-disturbance" && i + 1 < argc)
            inj.magDisturbance = parseRange(argv[++i]);
        else if (a == "--inject-invalid-quaternion" && i + 1 < argc)
            inj.invalidQuaternion = parseRange(argv[++i]);
        else if (a == "--compare")
            compareRequested = true;
        else if (a == "-h" || a == "--help")
        {
            printHelp();
            return 0;
        }
        else
        {
            std::fprintf(stderr, "unknown argument: %s\n", a.c_str());
            return 2;
        }
    }

    std::vector<ReplayRow> rows;
    try
    {
        rows = loadCsv(csvPath);
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 2;
    }
    if (rows.empty())
    {
        std::fprintf(stderr, "error: %s has no data rows\n", csvPath.c_str());
        return 2;
    }
    std::printf("loaded %zu samples from %s\n", rows.size(), csvPath.c_str());

    // Apply injections up front - simplest to reason about ("what will
    // this run do") and matches how a real bad-sensor-data window would
    // look to the pipeline: the raw sample itself is what's wrong, not
    // some separate override machinery layered on top of process().
    for (size_t i = 0; i < rows.size(); i++)
    {
        if (inj.staleDmp.contains((int)i))
        {
            // Handled at replay time (dmpFreshThisCycle=false, inflated
            // age) - see the per-row loop below. Nothing to mutate here.
        }
        if (inj.magDisturbance.contains((int)i))
        {
            rows[i].magBoat = rows[i].magBoat * 6.0; // well outside MagMonitorConfig's default plausible range/sudden-change threshold
        }
        if (inj.invalidQuaternion.contains((int)i))
        {
            rows[i].dmpQuat = Quaternion();
            rows[i].dmpQuat.w = 5.0;
            rows[i].dmpQuat.x = 5.0;
            rows[i].dmpQuat.y = 5.0;
            rows[i].dmpQuat.z = 5.0; // norm way outside DmpValidationConfig's default tolerance
        }
    }

    ImuCycleProcessor processor;
    CompareStats stats;
    unsigned long taskStartMs = rows.front().timestampMs;
    unsigned long lastFreshDmpMs = taskStartMs;
    Quaternion lastQuat = rows.front().dmpQuat;

    size_t index = 0;
    int loopsDone = 0;
    unsigned long prevTimestampMs = rows.front().timestampMs;

    auto processOne = [&]() -> bool {
        if (index >= rows.size())
        {
            loopsDone++;
            if (loopCount != 0 && loopsDone >= loopCount)
                return false;
            index = 0;
            lastFreshDmpMs = rows.front().timestampMs;
            lastQuat = rows.front().dmpQuat;
            prevTimestampMs = rows.front().timestampMs;
            std::printf("--- loop %d ---\n", loopsDone + 1);
        }
        ReplayRow &row = rows[index];
        bool forcedStale = inj.staleDmp.contains((int)index);
        bool freshThisCycle = !forcedStale && (index == 0 || row.dmpQuat.w != lastQuat.w || row.dmpQuat.x != lastQuat.x ||
                                                row.dmpQuat.y != lastQuat.y || row.dmpQuat.z != lastQuat.z);
        if (freshThisCycle)
        {
            lastFreshDmpMs = row.timestampMs;
            lastQuat = row.dmpQuat;
        }
        unsigned long age = forcedStale ? 10000 : (row.timestampMs - lastFreshDmpMs);

        double dtSec = (row.timestampMs > prevTimestampMs) ? (row.timestampMs - prevTimestampMs) / 1000.0 : 0.1;
        ImuCycleInput in = buildInput(row, freshThisCycle, age, taskStartMs, dtSec, dmpOk);
        ImuCycleOutput out = processor.process(in);

        printSample((int)index, row, out);
        if (compareRequested)
            updateCompareStats(stats, row, out);

        prevTimestampMs = row.timestampMs;
        index++;
        return true;
    };

    if (batch)
    {
        while (processOne())
        {
        }
        if (compareRequested)
            printCompareSummary(stats);
        return 0;
    }

    std::printf("interactive mode - type 'h' for help, 'q' to quit\n");
    bool running = false;
    std::string line;
    while (true)
    {
        if (!running)
        {
            std::printf("> ");
            std::fflush(stdout);
            if (!std::getline(std::cin, line))
                break;
            std::istringstream iss(line);
            std::string cmd;
            iss >> cmd;
            if (cmd == "q")
                break;
            if (cmd == "h")
            {
                printHelp();
                continue;
            }
            if (cmd == "s")
            {
                int n = 1;
                iss >> n;
                for (int i = 0; i < n; i++)
                    if (!processOne())
                        break;
                continue;
            }
            if (cmd == "r")
            {
                running = true;
                continue;
            }
            if (cmd == "speed")
            {
                iss >> speed;
                continue;
            }
            if (cmd == "loop")
            {
                iss >> loopCount;
                continue;
            }
            if (cmd == "compare")
            {
                printCompareSummary(stats);
                continue;
            }
            std::printf("unknown command '%s' - 'h' for help\n", cmd.c_str());
            continue;
        }

        // Running continuously - process one sample, pace by --speed,
        // then check (non-blocking) for a 'p' to pause.
        if (!processOne())
        {
            running = false;
            if (compareRequested)
                printCompareSummary(stats);
            continue;
        }
        if (speed > 0 && index > 0 && index < rows.size())
        {
            unsigned long dtMs = rows[index].timestampMs - rows[index - 1].timestampMs;
            std::this_thread::sleep_for(std::chrono::milliseconds(dtMs / (unsigned long)std::max(1, speed)));
        }
        if (stdinLineReady())
        {
            std::getline(std::cin, line);
            if (line == "p")
            {
                running = false;
                std::printf("paused at sample %zu\n", index);
            }
            else if (line == "q")
            {
                break;
            }
        }
    }

    return 0;
}
