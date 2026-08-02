# icm20948 debug replay tool

Runs recorded (or synthetic) sensor samples through the exact same
[`ImuCycleProcessor`](../../lib/icm20948pure/ImuCycleProcessor.h) the real
firmware task uses - not a reimplementation. It's a desktop-only debug
tool: it isn't part of any ESP32 board environment's source tree, so
there's no build flag or macro that could accidentally leave it in a
shipped firmware image - the code simply isn't reachable from any board
build.

## Why it lives under `test/`

The tool's source is at
[`test/test_replay_tool/test_main.cpp`](../../test/test_replay_tool/test_main.cpp),
not here. This project's PlatformIO setup can only discover
native-buildable (desktop) code via the `pio test` mechanism - per-env
`src_dir`/`test_dir` overrides don't work here (confirmed: an earlier
attempt at a dedicated `[env:icm20948_replay]` with its own `src_dir`
pulled in the entire Arduino-dependent firmware tree instead of the
intended file). It isn't a Unity test and asserts nothing; running it with
no arguments (as a full `pio test -e icm20948_native_test` run does) just
prints help and exits 0, so it doesn't fail the suite.

## Building

```sh
cd /path/to/esp32-nmea2000
pio test -e icm20948_native_test -f test_replay_tool --without-testing
```

The binary lands at `.pio/build/icm20948_native_test/program`. Run it
directly from there with real arguments (below) - `--without-testing`
just builds it without PlatformIO's test runner invoking it with no args.

**That `program` path is shared by every `test/test_*` suite** - a full
`pio test -e icm20948_native_test` run (no `-f` filter) rebuilds and
overwrites it once per suite, so after a full-suite run it's whatever
suite happened to build last, not this tool. Re-run the build command
above immediately before using the binary if you've run the full suite
in between.

## Usage

```
icm20948_replay <capture.csv> [options]
   or: icm20948_replay --generate <scenario> <output.csv> [durationSec=20] [rateHz=10]

  --batch                       run to completion non-interactively and exit
  --speed <n>                   playback speed multiplier for continuous runs (0 = as fast as possible)
  --loop <n>                    repeat the whole file n times (0 = infinite, default 1)
  --dmp-ok=true|false           whether DMP is considered active for this run (default true)
  --inject-stale-dmp R          force dmp_fresh=false + inflated age for row range R ("10" or "10:20")
  --inject-mag-disturbance R    scale the magnetometer reading for row range R
  --inject-invalid-quaternion R replace the DMP quaternion with a non-unit one for row range R
  --compare                     print actual-vs-expected diff summary against the CSV's own recorded output
```

Interactive commands (default mode, unless `--batch`): `s [n]` step,
`r` run continuously, `p` pause, `speed <n>`, `loop <n>`, `compare`, `q` quit.

### Load a real capture

Point it at a CSV from the firmware's runtime logger (Phase 2 - see
[`lib/icm20948pure/ImuDiagnostics.cpp`](../../lib/icm20948pure/ImuDiagnostics.cpp)
for the format, downloadable from the IMU web tab's Logging panel):

```sh
.pio/build/icm20948_native_test/program my_capture.csv --batch --compare
```

`--compare` diffs the freshly recomputed heading/source against what the
CSV's own `output_heading_deg`/`active_heading_source` columns recorded -
useful for checking whether a code change altered behavior on real
previously-captured data.

### Generate a synthetic fixture

Uses [`ImuSimulator`](../../lib/icm20948pure/ImuSimulator.h)'s scripted
scenarios, running each sample through a real `ImuCycleProcessor` as it
generates the file, so the recorded columns are ground truth (not an
approximation):

```sh
.pio/build/icm20948_native_test/program --generate level360Rotation my_fixture.csv 30 10
```

Available scenarios: `level360Rotation`, `rotation360With20DegHeel`,
`fixedHeadingChangingAttitude`, `hardIronOffset`,
`ellipticalSoftIronDistortion`, `suddenMagneticDisturbance`,
`slowGyroDrift`, `dmpHeadingDisagreement`, `headingWrapThroughNorth`,
`staleDmpOutput`, `quaternionNormError`, `magnetometerDropout` (the same
12 scripted scenarios `test_simulator_scenarios` uses).

## Fixture: `fixtures/level360Rotation.csv`

A committed, ready-to-use synthetic fixture: 30 seconds at 10 Hz (301
samples) of a level 360° rotation. Replaying it reproduces its own
recorded output exactly:

```sh
.pio/build/icm20948_native_test/program fixtures/level360Rotation.csv --batch --compare
# compared 301 samples: max heading diff 0.00 deg, mean 0.00 deg, 0/301 active-source mismatches
```

If a future code change makes that diff nonzero, it means production
heading-pipeline behavior changed (or this fixture needs regenerating) -
a coarse but genuine regression signal, since this tool runs the real
`ImuCycleProcessor`, not a copy of its logic.

## Known limitations

- Replaying a real captured CSV can't know for certain which cycles had a
  genuinely *fresh* DMP sample (that's not recorded) - it's reconstructed
  by comparing each row's quaternion to the previous row's (a changed
  quaternion is treated as fresh). This heuristic is exact for fixtures
  generated by `--generate` (which records true freshness) but is an
  approximation for a real device capture.
- Replays through *identity* calibration, not whatever calibration was
  active on the device that produced the CSV - the CSV's own
  `mag_boat_*`/`accel_boat_*`/`gyro_boat_*` columns are already
  pre-calibration by construction (see `ImuDiagnostics.h`), so this
  reproduces the RAW candidate sources' behavior, which is what matters
  for exercising the pipeline's validation/selection logic. Replaying
  through a *specific* saved calibration isn't wired up yet.
- `--speed`/`p` (pause during continuous run) rely on POSIX `select()` on
  stdin and aren't supported in a Windows build of this tool.
