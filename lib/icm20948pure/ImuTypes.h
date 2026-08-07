#pragma once
#include <stdint.h>
#include <math.h>

/*
  Shared types for the IMU heading pipeline. This header (and every other
  Imu*.h/.cpp file in this directory except GwIcm20948Task.* and
  GwIcm20948HardwareAdapter.*) has no Arduino/ESP32/hardware dependency on
  purpose - it compiles under both the real board envs and the
  icm20948_native_test host env (see platformio.ini), so the actual math can
  be unit-tested on a desktop.

  Boat coordinate frame (see doc/IcmHeadingArchitecture.md for the full
  writeup): X=forward (bow), Y=starboard, Z=down - a right-handed,
  NED-style frame. This matches the NMEA2000 sign conventions already
  confirmed against the ttlappalainen library's own doc comments: positive
  roll = heel to starboard, positive pitch = bow rising, positive rate of
  turn = turning to starboard. Heading (yaw) is compass convention: 0-360
  degrees (or 0-2pi radians), increasing clockwise from north.
*/

struct Vec3
{
    double x = 0, y = 0, z = 0;
    Vec3() {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3 &o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3 &o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    double dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
    double norm() const { return sqrt(x * x + y * y + z * z); }
};

// Hamilton convention, w is the scalar part. Represents a rotation from the
// boat frame to the reference (north/level) frame, same sense the software
// fusion filter produces.
struct Quaternion
{
    double w = 1, x = 0, y = 0, z = 0;
};

// What the user has configured (icmHeadingMode). "Auto" isn't a source in
// itself - it's a policy that picks among the software sources at runtime.
enum class HeadingSourceMode : uint8_t
{
    DiagnosticOnly = 0,
    SoftwareCompass = 2,
    SoftwareFusion = 3,
    Auto = 4,
};

// Which source is ACTUALLY driving the output heading right now - always
// concrete, never "Auto" (that's resolved to one of these by the selection
// policy in ImuHeadingSource).
enum class HeadingSource : uint8_t
{
    None = 0,
    SoftwareCompass = 2,
    SoftwareFusion = 3,
};

enum class HeadingQuality : uint8_t
{
    Invalid = 0,
    Poor = 1,
    Good = 2,
};

// Bitmask - more than one reason can apply at once, and the exact set is
// worth surfacing in diagnostics rather than collapsing to a single code.
enum HeadingRejectReason : uint32_t
{
    HR_NONE = 0,
    HR_INITIALIZING = 1u << 0,
    HR_MAG_FIELD_LOW = 1u << 3,
    HR_MAG_FIELD_HIGH = 1u << 4,
    HR_MAG_FIELD_CHANGE = 1u << 5,
    HR_CALIBRATION_INVALID = 1u << 7,
    HR_SENSOR_READ_ERROR = 1u << 8,
    HR_FUSION_COMPASS_DISAGREE = 1u << 11,
    HR_MAG_INVALID = 1u << 12,
};

// The 24 valid right-angle mountings of a board with two horizontal
// reference axes (front-panel-forward, front-panel-down, etc.) - see
// ImuCoordinateTransform.h for how this maps to a signed permutation
// matrix. Forward/Starboard/Aft/Port are the common flat-mount cases;
// the *Up/*Down variants cover the sensor mounted on a vertical
// bulkhead/panel.
enum class MountOrientation : uint8_t
{
    Forward = 0,
    Starboard = 1,
    Aft = 2,
    Port = 3,
    ForwardFlipped = 4, // upside-down variants (Z flipped relative to the flat mounts)
    StarboardFlipped = 5,
    AftFlipped = 6,
    PortFlipped = 7,
    ForwardUp = 8, // sensor's flat side faces forward/aft/port/starboard
    ForwardDown = 9, // (mounted on a vertical bulkhead), remaining 16 of 24
    AftUp = 10,
    AftDown = 11,
    PortUp = 12,
    PortDown = 13,
    StarboardUp = 14,
    StarboardDown = 15,
    ForwardUpFlipped = 16,
    ForwardDownFlipped = 17,
    AftUpFlipped = 18,
    AftDownFlipped = 19,
    PortUpFlipped = 20,
    PortDownFlipped = 21,
    StarboardUpFlipped = 22,
    StarboardDownFlipped = 23,
};

// One fully-computed cycle's worth of output - built once per loop from
// whichever candidate sources were valid, then used to emit PGN
// 127257/127250/127251 so all three are always consistent with each other
// (never mixing this cycle's roll/pitch with a stale heading).
struct ImuSolution
{
    uint32_t timestampMs = 0;
    uint8_t sid = 0;

    double rollRad = 0;
    double pitchRad = 0;
    double headingRad = 0; // magnetic heading
    double rateOfTurnRadPerSec = 0;

    bool attitudeValid = false;
    bool headingValid = false;
    bool rateOfTurnValid = false;

    HeadingSource headingSource = HeadingSource::None;
    HeadingQuality quality = HeadingQuality::Invalid;
    uint32_t rejectionFlags = HR_NONE;
};
