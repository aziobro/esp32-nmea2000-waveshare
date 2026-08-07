// Debug-only CSV replay tool for the icm20948 heading pipeline.
//
// Runs recorded (or synthetic) raw sensor samples through the same
// ImuCycleProcessor the firmware uses. This desktop-only utility is not
// part of any ESP32 firmware image; it lives under test/ only because the
// project's PlatformIO native environment discovers host tools that way.
#include "ImuCycleProcessor.h"
#include "ImuDiagnostics.h"
#include "ImuSimulator.h"

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
#include <unordered_map>
#include <vector>

#ifndef _WIN32
#include <sys/select.h>
#include <unistd.h>
#endif

namespace
{
struct ReplayRow
{
    unsigned long timestampMs = 0;
    Vec3 accelBoat, gyroBoat, magBoat;
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

std::unordered_map<std::string, size_t> headerMap(const std::vector<std::string> &header)
{
    std::unordered_map<std::string, size_t> out;
    for (size_t i = 0; i < header.size(); i++)
        out[header[i]] = i;
    return out;
}

size_t requireColumn(const std::unordered_map<std::string, size_t> &columns, const char *name, const std::string &path)
{
    auto it = columns.find(name);
    if (it == columns.end())
        throw std::runtime_error(path + ": missing required column '" + name + "'");
    return it->second;
}

double optionalDouble(const std::vector<std::string> &fields, const std::unordered_map<std::string, size_t> &columns,
                      const char *name, double fallback)
{
    auto it = columns.find(name);
    if (it == columns.end() || it->second >= fields.size() || fields[it->second].empty())
        return fallback;
    return std::stod(fields[it->second]);
}

std::string optionalString(const std::vector<std::string> &fields, const std::unordered_map<std::string, size_t> &columns,
                           const char *name)
{
    auto it = columns.find(name);
    if (it == columns.end() || it->second >= fields.size())
        return "";
    return fields[it->second];
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
    auto columns = headerMap(header);
    const size_t ts = requireColumn(columns, "timestamp_ms", path);
    const size_t ax = requireColumn(columns, "accel_boat_x", path);
    const size_t ay = requireColumn(columns, "accel_boat_y", path);
    const size_t az = requireColumn(columns, "accel_boat_z", path);
    const size_t gx = requireColumn(columns, "gyro_boat_x", path);
    const size_t gy = requireColumn(columns, "gyro_boat_y", path);
    const size_t gz = requireColumn(columns, "gyro_boat_z", path);
    const size_t mx = requireColumn(columns, "mag_boat_x", path);
    const size_t my = requireColumn(columns, "mag_boat_y", path);
    const size_t mz = requireColumn(columns, "mag_boat_z", path);

    std::vector<ReplayRow> rows;
    std::string line;
    int lineNo = 1;
    while (std::getline(f, line))
    {
        lineNo++;
        if (line.empty())
            continue;
        std::vector<std::string> fields = splitCsvLine(line);
        if (fields.size() < header.size())
            throw std::runtime_error(path + ":" + std::to_string(lineNo) + ": too few CSV fields");

        ReplayRow row;
        row.timestampMs = std::stoul(fields[ts]);
        row.accelBoat = Vec3(std::stod(fields[ax]), std::stod(fields[ay]), std::stod(fields[az]));
        row.gyroBoat = Vec3(std::stod(fields[gx]), std::stod(fields[gy]), std::stod(fields[gz]));
        row.magBoat = Vec3(std::stod(fields[mx]), std::stod(fields[my]), std::stod(fields[mz]));
        row.expectedOutputHeadingDeg = optionalDouble(fields, columns, "output_heading_deg", -1.0);
        row.expectedActiveSource = optionalString(fields, columns, "active_heading_source");
        row.expectedHeadingQuality = optionalString(fields, columns, "heading_quality");
        rows.push_back(row);
    }
    return rows;
}

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
    IndexRange magDisturbance;
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
    if (row.expectedOutputHeadingDeg < 0 || !out.headingValid)
        return;
    stats.compared++;
    double diff = circularDiffDeg(out.headingDeg, row.expectedOutputHeadingDeg);
    stats.sumHeadingDiffDeg += diff;
    if (diff > stats.maxHeadingDiffDeg)
        stats.maxHeadingDiffDeg = diff;
    if (!row.expectedActiveSource.empty() && sourceName(out.headingSource) != row.expectedActiveSource)
        stats.sourceMismatches++;
}

void printCompareSummary(const CompareStats &stats)
{
    std::printf("\n--- compare actual (this code) vs expected (recorded in the CSV) ---\n");
    if (stats.compared == 0)
    {
        std::printf("no comparable samples\n");
        return;
    }
    std::printf("compared %d samples: max heading diff %.2f deg, mean %.2f deg, %d/%d active-source mismatches\n",
                stats.compared, stats.maxHeadingDiffDeg, stats.sumHeadingDiffDeg / stats.compared,
                stats.sourceMismatches, stats.compared);
}

ImuCycleInput buildInput(const ReplayRow &row, unsigned long taskStartMs, double dtSec)
{
    ImuCycleInput in;
    in.accelBoat = row.accelBoat;
    in.gyroBoat = row.gyroBoat;
    in.magBoat = row.magBoat;
    in.dtSec = dtSec;
    in.nowMs = row.timestampMs;
    in.taskStartMs = taskStartMs;
    in.headingMode = HeadingSourceMode::Auto;
    in.transitionMs = 1000;
    return in;
}

bool stdinLineReady()
{
#ifdef _WIN32
    return false;
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
    if (name == "headingWrapThroughNorth") return headingWrapThroughNorth(t);
    if (name == "magnetometerDropout") return magnetometerDropout(t);
    throw std::runtime_error("unknown scenario '" + name + "'");
}

void generateFixture(const std::string &scenario, const std::string &outPath, double durationSec, double rateHz)
{
    std::ofstream f(outPath);
    if (!f)
        throw std::runtime_error("cannot create " + outPath);
    f << ImuDiagnostics::csvHeader() << "\n";

    ImuCycleProcessor processor;
    double dt = 1.0 / rateHz;
    int n = (int)(durationSec / dt);
    unsigned long taskStartMs = 0;

    for (int i = 0; i <= n; i++)
    {
        double t = i * dt;
        SimulatorState state = scenarioState(scenario, t);
        SimulatedSample sample = ImuSimulator::generateSample(state);
        unsigned long tsMs = (unsigned long)(t * 1000.0);

        ReplayRow row;
        row.timestampMs = tsMs;
        row.accelBoat = sample.accelG;
        row.gyroBoat = sample.gyroDegPerSec;
        row.magBoat = sample.magRaw;

        ImuCycleOutput out = processor.process(buildInput(row, taskStartMs, dt));

        DiagnosticSample diag;
        diag.timestampMs = tsMs;
        diag.sampleSequence = i;
        diag.accelRaw = sample.accelG;
        diag.gyroRaw = sample.gyroDegPerSec;
        diag.magRaw = sample.magRaw;
        diag.accelBoat = sample.accelG;
        diag.gyroBoat = sample.gyroDegPerSec;
        diag.magBoat = sample.magRaw;
        diag.magCorrected = out.magCorrected;
        diag.magMagnitude = out.magMagnitude;
        diag.compassHeadingDeg = out.rawCompassHeadingDeg;
        diag.fusionHeadingDeg = out.rawFusionHeadingDeg;
        diag.outputHeadingDeg = out.headingValid ? out.headingDeg : -1.0;
        diag.rateOfTurnDegPerSec = out.rotDegPerSec;
        diag.activeSource = out.headingSource;
        diag.headingQuality = out.headingQuality;
        diag.rejectionFlags = out.rejectionFlags;

        char buf[800];
        if (ImuDiagnostics::formatCsvRow(diag, buf, sizeof(buf)) < 0)
            throw std::runtime_error("generated diagnostic row did not fit buffer");
        f << buf << "\n";
    }
    std::printf("wrote %d samples to %s (scenario=%s, duration=%.1fs, rate=%.1fHz)\n",
                n + 1, outPath.c_str(), scenario.c_str(), durationSec, rateHz);
}

void printHelp()
{
    std::printf(
        "icm20948_replay - debug-only CSV replay through the real heading pipeline\n\n"
        "usage: icm20948_replay <capture.csv> [options]\n"
        "   or: icm20948_replay --generate <scenario> <output.csv> [durationSec=20] [rateHz=10]\n"
        "       scenarios: level360Rotation, rotation360With20DegHeel, fixedHeadingChangingAttitude,\n"
        "                  hardIronOffset, ellipticalSoftIronDistortion, suddenMagneticDisturbance,\n"
        "                  slowGyroDrift, headingWrapThroughNorth, magnetometerDropout\n"
        "  --batch                 run to completion non-interactively and exit\n"
        "  --speed <n>             playback speed multiplier for continuous runs (0 = as fast as possible, default 0)\n"
        "  --loop <n>              repeat the whole file n times (0 = infinite, default 1)\n"
        "  --inject-mag-disturbance R   scale the magnetometer reading for row range R\n"
        "  --compare               print actual-vs-expected diff summary against the CSV's own recorded output\n\n"
        "Interactive commands (default mode, unless --batch):\n"
        "  s [n]      step n samples (default 1)\n"
        "  r          run continuously to completion\n"
        "  p          pause\n"
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
    bool compareRequested = false;
    Injections inj;

    for (int i = 2; i < argc; i++)
    {
        std::string a = argv[i];
        if (a == "--batch")
            batch = true;
        else if (a == "--speed" && i + 1 < argc)
            speed = std::stoi(argv[++i]);
        else if (a == "--loop" && i + 1 < argc)
            loopCount = std::stoi(argv[++i]);
        else if (a == "--inject-mag-disturbance" && i + 1 < argc)
            inj.magDisturbance = parseRange(argv[++i]);
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

    for (size_t i = 0; i < rows.size(); i++)
    {
        if (inj.magDisturbance.contains((int)i))
            rows[i].magBoat = rows[i].magBoat * 6.0;
    }

    ImuCycleProcessor processor;
    CompareStats stats;
    unsigned long taskStartMs = rows.front().timestampMs;
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
            prevTimestampMs = rows.front().timestampMs;
            std::printf("--- loop %d ---\n", loopsDone + 1);
        }
        ReplayRow &row = rows[index];
        double dtSec = (row.timestampMs > prevTimestampMs) ? (row.timestampMs - prevTimestampMs) / 1000.0 : 0.1;
        ImuCycleOutput out = processor.process(buildInput(row, taskStartMs, dtSec));

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

        if (!processOne())
        {
            running = false;
            if (compareRequested)
                printCompareSummary(stats);
            continue;
        }
        if (speed > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / speed));
        if (stdinLineReady())
        {
            std::getline(std::cin, line);
            if (line == "p")
                running = false;
            else if (line == "q")
                break;
        }
    }
    if (compareRequested)
        printCompareSummary(stats);
    return 0;
}
