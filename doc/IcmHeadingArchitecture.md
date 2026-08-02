# ICM-20948 heading pipeline architecture

This documents the IMU heading/attitude pipeline in `lib/icm20948pure`
(pure math, no Arduino/ESP32 dependency, unit-tested under
`icm20948_native_test` - see `lib/icm20948task/platformio.ini`) and
`lib/icm20948task` (orchestration + the ICM_20948 hardware adapter).

## Why this exists

The original implementation sent NMEA2000 heading (PGN 127250) straight
from the ICM-20948's onboard DMP fusion's yaw output whenever DMP was
enabled, correcting it with only a fixed offset. That bypassed the
magnetometer hard-iron/soft-iron calibration entirely once DMP was on
(a fixed offset can't correct hard-iron, soft-iron, axis-mapping, tilt,
or heading-dependent error), and never validated the DMP sample itself -
a stale FIFO read or a corrupted quaternion would be transmitted exactly
as confidently as a good one.

The rebuilt pipeline treats DMP yaw as one candidate heading source among
several (software tilt-compensated compass, software 9-axis fusion),
each independently computed every cycle and validated before
`ImuHeadingSource` selects/blends the active one.

## Coordinate systems

- **Boat frame** (the pipeline's own working frame): X = forward (bow),
  Y = starboard, Z = down. Right-handed, NED-style. Matches the NMEA2000
  sign conventions already confirmed against the ttlappalainen library's
  own doc comments: positive roll = heel to starboard, positive pitch =
  bow rising, positive rate of turn = turning to starboard.
- **Sensor frame**: whatever the ICM-20948's native chip axes are once
  mounted. `ImuCoordinateTransform` maps sensor frame -> boat frame via a
  signed 3x3 rotation selected by `icmOrientation`
  (`MountOrientation`). `Forward` (the default) is the identity
  transform, reproducing the pre-rewrite code's implicit assumption
  (sensor axes already aligned with boat axes) exactly.
- **DMP quaternion frame**: matches boat frame in, reference (level/
  north) frame out - i.e. the quaternion represents a rotation *from*
  boat frame *to* the reference frame, the same convention
  `ImuQuaternion` and `ImuFusion`'s Mahony filter both use internally.
- **NMEA2000 Attitude (127257) / Rate of Turn (127251)**: radians, same
  boat-frame sign convention as above - no additional conversion needed
  at the NMEA boundary beyond degrees-to-radians.

### Mounting orientations

24 valid right-angle mountings exist (one per way to orient a cube).
`Forward`/`Starboard`/`Aft`/`Port` (flat, horizontal mounts) are
hand-derived and individually verified against the boat's own heading
convention. The remaining 20 (tilted/vertical/upside-down mounts) are
generated exhaustively - every signed axis permutation with determinant
+1 is a valid rotation, and there are exactly 24 - guaranteeing all are
valid, distinct rotations, but **which exact enum value corresponds to
which real physical mounting is not yet confirmed against hardware**.
See `doc/IcmPhysicalTestProcedure.md`.

## Processing order

Every cycle, in this order:

1. Read raw accelerometer, gyro, magnetometer from the hardware adapter.
2. Transform all three into boat frame (`ImuCoordinateTransform`).
3. Apply sensor calibration (`ImuCalibrationOps` - hard/soft-iron on
   mag, bias+scale on accel, bias on gyro).
4. Determine roll/pitch (from DMP if active, else the plain
   accelerometer tilt formula on the calibrated, boat-frame accel
   vector).
5. Compute all three heading candidates:
   - **DMP**: the chip's own fused yaw, if DMP is enabled and a fresh
     (or recent-enough) FIFO sample exists.
   - **Software compass**: `ImuCompass::rawHeadingDeg` - tilt-
     compensated, using the calibrated magnetometer and the roll/pitch
     from step 4.
   - **Software fusion**: `MahonyFusion`, running continuously every
     cycle regardless of which mode is active (so it's already
     converged if the user switches to it), fed boat-frame calibrated
     accel/gyro/mag.
6. Validate DMP (`DmpValidator`): staleness, quaternion finiteness/norm,
   startup convergence window, disagreement with the software compass,
   implausible sudden jumps - returns a `HeadingRejectReason` bitmask,
   not a single pass/fail bit, so the exact reason is always visible in
   diagnostics (`/api/user/icm20948Task/data`, the IMU web tab).
7. Monitor the magnetic field (`ImuMagMonitor`) for disturbance
   (out-of-range magnitude, abrupt change, sustained deviation from a
   reference) with hysteresis, feeding into DMP/compass candidate
   quality.
8. Select/blend the active source (`HeadingSourceSelector`) per
   `icmHeadingMode`:
   - `dmp` / `software_compass` / `software_9axis_fusion`: use only that
     source, invalid if it isn't currently valid (no automatic
     fallback to a different one).
   - `auto`: prefer fusion, then DMP, then compass, then invalid.
     Source changes blend circularly over `icmSrcTransMs` rather than
     jumping instantly.
   - `diagnostic_only`: computed identically to `auto` for display, but
     the orchestration layer never transmits PGN 127250 in this mode.
9. Apply the fixed mounting offset (`icmHdgOff`) and invert
   (`icmHdgInv`) to the *selected* heading - uniformly, regardless of
   which source produced it, matching the pre-rewrite code's structure
   exactly for the DMP case.
10. Apply the deviation table (`ImuDeviationTable`), if enabled
    (`icmDevEnable`).
11. Apply the heading filter (`ImuHeadingFilter`), if enabled
    (`icmHdgFiltEn`) - always via `ImuAngleMath`'s circular operations,
    never plain arithmetic (which breaks near the 0/360 wrap).
12. Emit PGN 127257 (Attitude), 127250 (Magnetic Heading, only if
    `icmSendHdg` is on and a valid heading resulted), and 127251 (Rate
    of Turn) - all from the same cycle's computed values, never mixing
    a stale heading into a fresh attitude message.

Magnetic variation is never applied - the heading output is always
magnetic, never claimed as true, matching the pre-rewrite code and the
project's own instruction not to synthesize true heading.

## Config migration

Every pre-rewrite config key keeps its exact name, default, and meaning.
`ImuCalibrationOps::migrateFromLegacy()` builds the new versioned
calibration model directly from `icmMagXOff`/`icmMagYOff`/`icmHdgOff`,
verified by test to reproduce the old hard-iron-only formula exactly
(identity soft-iron matrix, Z axis never hard-iron corrected, same as
before). All new config defaults to values that reproduce today's
behavior - most notably `icmHeadingMode` defaults to `dmp`, not `auto`:
introducing software fusion as a new candidate source should not change
a currently-deployed, already-tuned unit's output the moment this
firmware update lands. `icmSendHdg` is untouched and still defaults off.

New config item names are capped at 15 characters - an NVS key-length
constraint enforced by this project's own build script
(`extra_script.py`), not obvious until the build failed on it. Some
names were abbreviated accordingly (e.g. `icmHeadingFilterEnable` ->
`icmHdgFiltEn`).

## What's fully implemented and tested

Coordinate transform, calibration model (bias+matrix), tilt-compensated
compass, quaternion math, Mahony fusion, DMP validation, source
selection/blending, magnetic disturbance monitoring, circular heading
filter, stationary gyro-bias calibration engine, deviation table, and
the sensor simulator (12 scripted scenarios) - all pure, all under
`icm20948_native_test`, all wired into the real task.

## What's deferred

- **Full soft-iron matrix / 3D calibration**: the model
  (`ImuCalibration`) and its application (`ImuCalibrationOps::applyMag`)
  fully support an arbitrary 3x3 matrix and are tested against known
  hard-iron, unequal-scale, and full soft-iron cases - but nothing in
  the web UI yet lets a user *produce* a non-identity matrix (no 2D
  boat-swing calibration engine or offline ellipsoid-fit tool wired up
  yet). The task currently only ever populates a hard-iron bias from the
  legacy two config fields.
- **Compass deviation table web editor**: `ImuDeviationTable` is fully
  implemented and tested; nothing currently lets a user add entries to
  it from the web UI, so `icmDevEnable` has no visible effect yet even
  when turned on (an empty table applies zero correction everywhere).
- **CSV capture/download logging, debug replay mode, offline Python
  calibration tool**: not yet built.
- **Stationary gyro calibration UI**: `GyroCalEngine` is implemented and
  tested; nothing in the task/web UI currently drives it or persists a
  learned gyro bias into the calibration model.

## Next physical test

See `doc/IcmPhysicalTestProcedure.md` and
`doc/IcmHardwareTestWorksheet.md`/`.csv`.
