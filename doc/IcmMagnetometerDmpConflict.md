# Magnetometer/DMP I2C_SLV0 conflict - root cause and fix plan

Found during hardware testing on the boat 2026-08-05/06, after the IMU
replacement unit.

**2026-08-06: fix implemented AND bench-verified working on the real IMU
unit over USB/WiFi.** See "Fix implemented" and "Bench verification"
sections at the bottom. A real tumble-capture (device physically rotated
through orientations, not just stationary) and the ellipsoid-fit quality
gate retest are still open - see "Still needed" at the very end.

## Symptom

Every magnetometer calibration capture that session (~10 attempts, across
underway/at-mooring, engine on/off, multiple deck locations, continuous and
stop-start tumbling technique, both `icmOrientation` settings tried) landed
in the same 16-23% ellipsoid-fit residual band, never clearing the offline
tool's 15% quality gate. Neither relocating, changing technique, removing
worn metal from the person tumbling it, nor switching to a simpler
pypilot-style sphere-only fit (no soft-iron matrix) made a real difference.
That flatness across every variable we could think to change was the tell
that this wasn't an environmental or technique problem.

## Evidence

Every single capture that session, without exception:

```
X: 3-4 unique values only, range ~76-115 uT, step size exactly 38.4 uT
Y: 150-290 unique values, full natural range (~100-150 uT)
Z: 150-290 unique values, full natural range (~90-115 uT)
```

38.4 uT / 0.15 uT-per-LSB (the AK09916's fixed scale factor, see
`ICM_20948::getMagUT()` in the SparkFun library) = exactly 256 = 2^8. The
X-axis magnetometer reading only ever changes in whole-high-byte steps -
the low byte is not contributing real information. Y and Z show normal
full-resolution jitter throughout. This is present in captures from
*before* any of that session's diagnostic work started too, going back to
the very first capture - it is not something introduced by the orientation
or watchdog fixes made that session.

## Root cause

`GwIcm20948HardwareAdapter::readAGMT()` (called every cycle, both for
regular sensor reads and to feed `readMagRaw()`) calls `imu.getAGMT()`,
which calls `ICM_20948_get_agmt()` in
`.pio/libdeps/*/SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino
Library/src/util/ICM_20948_C.c` (~line 1055). That function reads a fixed
23-byte block starting at `ACCEL_XOUT_H`: bytes 0-13 are accel/gyro/temp
(stable, dedicated registers, unaffected by anything below), bytes 14-22
are the `EXT_SLV_SENS_DATA` shadow registers that the ICM-20948's internal
I2C_SLV0 peripheral auto-populates from the AK09916 magnetometer over the
auxiliary bus. `ICM_20948_get_agmt()` has a **hardcoded assumption** about
what's in those shadow bytes: little-endian, 9 bytes, starting with the
ST1 status register (`pagmt->mag.axes.x = (buff[16]<<8)|buff[15]`, etc.,
explicitly commented "Mag data is read little endian").

That assumption is only true if I2C_SLV0 is still configured the way
`startupMagnetometer()` (the non-DMP path) set it up. But `startupDMP()`
in `ICM_20948.cpp` (~line 1475-1509) **reconfigures the same I2C_SLV0
peripheral** to a completely different, reverse-engineered layout for DMP
compatibility - the comments there are the library authors' own admission
this is undocumented:

> "The datasheet does not define what registers 0x04 to 0x0C contain.
> There is definitely some secret sauce in here... We had to examine the
> I2C traffic between the master and the AK09916 ... to discover this."

That DMP-mode layout is big-endian, 10 bytes, starts at reserved register
`RSV2` (0x03) instead of `ST1`, with `I2C_SLV0_GRP` (byte-pairing) and
`I2C_SLV0_BYTE_SW` (byte-swap) both set - copied from InvenSense's
confidential `inv_icm20948_resume_akm` reference code, not from any public
datasheet.

Our task calls `hw.initDmp()` (which calls `startupDMP()`) once at startup,
before the main loop begins - see `GwIcm20948Task.cpp`'s
`runIcm20948Task()`, the `dmpOk = useDmp && hw.initDmp(logger);` line. Every
`readAGMT()` call for the rest of that boot therefore reads magnetometer
shadow bytes in the DMP-mode layout, but decodes them assuming the
non-DMP layout. Accel/gyro (fixed dedicated registers, not routed through
I2C_SLV0 at all) are completely unaffected - which is exactly why the
whole roll/pitch/DMP-orientation debugging that same session worked out
cleanly and only the magnetometer stayed broken. Depending on exactly how
the byte-swap/grouping misalignment lines up, some resulting bytes still
happen to vary smoothly sample-to-sample (Y, Z); others land on the
slower-changing bytes of the wrong quantity (X).

## Fix

The DMP FIFO packet format already carries a natively-fused,
DMP-internally-correct compass reading - no raw-register parsing needed,
same mechanism already used for the quaternion:

- `util/ICM_20948_DMP.h`: `DMP_header_bitmap_Compass_Calibr = 0x0020`
  ("32-bit calibrated compass"), `icm_20948_DMP_Compass_Calibr_Bytes = 12`,
  and the `icm_20948_DMP_data_t` union's `Compass_Calibr.Data.{X,Y,Z}`
  (`int32_t`, "unit is uT scaled by 2^16").
- Enable it via `imu.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD)`
  in `GwIcm20948HardwareAdapter::initDmp()`, alongside the existing
  `enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION)` call.
- Read it via the **same** `imu.readDMPdataFromFIFO(&data)` call
  `readDmpQuaternion()` already makes (check
  `data.header & DMP_header_bitmap_Compass_Calibr`, matching the existing
  `DMP_header_bitmap_Quat9` pattern) - add a sibling method, e.g.
  `readDmpCompass()`, rather than complicating `readDmpQuaternion()`.
- Replace the `hw.readMagRaw()` call in `GwIcm20948Task.cpp`'s main loop
  with the new DMP-sourced value when `dmpOk` (mirroring how
  `readDmpQuaternion()` already gates on `dmpOk`/`haveDmpSample` for
  roll/pitch) - keep `getAGMT()`-based `readMagRaw()` as the fallback for
  when DMP isn't active, since that path's raw-register parsing is correct
  in that case.
- Handle `Compass_Off`/`Compass_On` status bits (`icm_20948_DMP_data_t`,
  ~line 504-505) the same way `dmpStat`/FIFO-empty is already handled for
  the quaternion path.
- Scale: divide by 2^16 to get uT (`Compass_Calibr` is "scaled by 2^16"
  per the struct comment) - verify against a few known-good bench readings
  before trusting it, the same way `readDmpQuaternion()`'s Q30 scaling was
  verified.
- Add native tests for the new parsing (`test_diagnostics` or a new
  `test_dmp_compass` suite), matching this project's existing test
  coverage for `readDmpQuaternion()`-adjacent code - this repo has a
  documented habit of finding real bugs in exactly this kind of test
  (see the quaternion-rotation fix from the same session, where the first
  version of `test_rotateDmpQuaternion_consistent_with_matrix_for_all_24_orientations`
  caught a flawed test invariant, not a real implementation bug - worth
  writing the test carefully rather than trusting the derivation alone).
- Once this lands, redo the magnetometer tumble capture and confirm X
  shows full-resolution jitter (150+ unique values over a ~40s capture,
  matching Y/Z) before trusting the ellipsoid fit's residual number again.

## Other loose ends from the same session (lower priority, not forgotten)

- **`/api/setConfig` (`handleConfigRequestData` in `src/main.cpp`) is
  broken** - failed identically 3 times that session (empty HTTP reply,
  no restart, no persisted change) for both `logLevel` and `icmPitchInv`.
  Confirmed NOT an auth/hash problem (`/api/checkPass` independently
  verified the hash was correct each time) and NOT a device crash (the
  rest of the device stayed fully responsive throughout - `/api/status`
  polled every second for 20+ seconds during one failed attempt, zero
  drops). Worked around for the invert flags via a new
  `calControl?action=setInvert` endpoint added to
  `GwIcm20948CalControlTask.cpp`, which reuses the same
  `config->setValue()`/`updateValue()` calls the working
  `calControl?action=import` path already used reliably for
  `icmOrientation` - so the underlying config-write primitives are fine,
  the bug is specific to the general form-POST handler. Root cause not
  diagnosed. Worth a proper fix since this also breaks the normal
  Config-page save flow in a browser, not just API calls.
- **Calibration save doesn't auto-restart** - `persistCalibration()` in
  `GwIcm20948CalControlTask.cpp` writes to NVS via `updateValue()` but
  never calls `delayedRestart()` like every other config-writing path in
  this project does, so a save is silently invisible to the running
  device until a manual reboot. One-line fix: add `delayedRestart()` at
  the end of `persistCalibration()`.
- **`icmSendHdg` was left on** with no valid magnetometer calibration
  active (`magCalibration.valid: false`) - explicit user decision that
  session, not an oversight, but worth revisiting before trusting PGN
  127250 for navigation. Also revisit once the DMP compass fix above
  lands, since `icmHeadingMode` was set to `software_9axis_fusion`
  (not the default `dmp`) that session - re-evaluate whether `dmp` mode
  is viable again once its magnetometer input is actually correct.
- **Gyroscope calibration not done** - needs the boat stationary
  (dockside/mooring, which was available near the end of that session but
  by then the priority had shifted to the magnetometer bug above).
- **Physical test procedure steps 11-21** (tilt test, engine/interference
  comparisons, trusted-compass comparison, deviation table, Garmin PGN
  verification) still open - see `doc/IcmPhysicalTestProcedure.md`. Steps
  5-7 (roll/pitch/heading sign) passed with `icmOrientation=Starboard`,
  `icmPitchInv=true`, `icmHdgInv=true` (carried over, reconfirmed correct
  after the orientation change).
- **I2C hang watchdog** (`icm20948WatchdogTaskEntry` in
  `GwIcm20948Task.cpp`) - fixed and confirmed working that session, no
  further action needed. Root cause was a Qwiic cable connection issue
  under physical handling combined with the ESP32 Arduino `Wire` library
  not honoring its own `setTimeOut()` for the specific fault mode that
  produced; the watchdog converts any recurrence into a ~10s
  self-recovering reboot instead of a permanent hang.

## Fix implemented (2026-08-06)

Followed the plan above with one deliberate deviation, explained below.
Compiles clean for `waveshare-esp32s3-rs485-can` and all 185 existing
native (`pio test -e icm20948_native_test`) cases still pass. **Not yet
bench-verified against real hardware** - that's the next real-world step.

- `GwIcm20948HardwareAdapter::initDmp()` now also calls
  `enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD)` and
  `setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0)` (0 = max rate, same as the
  existing Quat9 call) - without an explicit ODR the compass-calibr FIFO
  field wasn't confirmed to output at a usable rate by default, so this
  mirrors the existing Quat9 ODR override rather than trusting whatever
  the DMP firmware image's own built-in default is.
- **Deviation from the plan's literal wording**: the plan said to "add a
  sibling method `readDmpCompass()`" that reads `data.header &
  DMP_header_bitmap_Compass_Calibr` "via the same
  `imu.readDMPdataFromFIFO(&data)` call `readDmpQuaternion()` already
  makes." Taken completely literally (two independent methods, each with
  its own do-while FIFO-drain loop, both called once per cycle) this
  would be broken: `readDmpQuaternion()`'s own do-while already drains the
  FIFO down to `FIFONoDataAvail` every cycle, so a *second*, separate
  `readDMPdataFromFIFO()` call afterwards would always find nothing left
  and report false forever - not a hypothetical, it follows directly from
  how the existing drain loop's `while (dmpStat ==
  ICM_20948_Stat_FIFOMoreDataAvail)` condition works. SparkFun's own
  `Example9_DMP_MultipleSensors.ino` confirms the actual FIFO framing:
  ONE `readDMPdataFromFIFO()` call returns a single frame whose `header`
  can have *multiple* bits set at once (that example checks
  `Quat6`/`Accel`/`Gyro`/`Compass` bits all on the same `data` from one
  call) - Quat9 and Compass_Calibr routinely arrive in the very same
  frame here, not separate ones. Implemented instead: `readDmpQuaternion()`
  keeps its existing signature/behavior (no complication for existing
  callers, matching the plan's intent there) but its *existing* do-while
  loop also checks `DMP_header_bitmap_Compass_Calibr` on the same `data`
  struct it's already holding and caches any hit into a private
  `lastDmpCompassRaw`/`haveDmpCompassSample` pair. The new
  `readDmpCompass(Vec3&)` is a pure accessor - it returns that cached
  value and does **not** touch the FIFO itself, and its header comment
  says explicitly it must be called after `readDmpQuaternion()` in the
  same cycle. Net effect matches the plan's intent (a sibling method,
  `readDmpQuaternion()`'s own signature unchanged) without the double-drain
  bug the literal reading would have caused.
- `GwIcm20948Task.cpp`'s main loop: added persistent
  `lastDmpMagRaw`/`haveDmpCompassSample` state (mirrors
  `lastDmpQuat`/`haveDmpSample` - "last known good", not "this cycle
  only", since the compass field doesn't necessarily land in every single
  drained frame). The DMP-sample block (previously positioned after the
  calibration-engine feed calls) was **moved earlier**, to right after
  `magBoat` is first computed from `readMagRaw()` and before
  `calControl.feedGyroSample()`/`feedMagSample()` run - not mentioned in
  the plan, but necessary: `feedMagSample()` feeds the live
  Calibration-panel hard-iron tracking (the same mechanism this whole bug
  was originally found through), and if the DMP-corrected `magBoat`
  override happened after that call (as a naive read of the plan's file
  layout might suggest), the live calibration UI would keep calibrating
  against the same corrupted raw values this fix is meant to eliminate.
  With the reorder, `magBoat` is DMP-corrected (when `dmpOk &&
  haveDmpCompassSample`) before *anything* downstream consumes it -
  calibration engines, the CSV diagnostic capture, and the heading
  pipeline all see the same corrected value.
- Did **not** implement a `Compass_Off`/`Compass_On` status-bit check
  (`icm_20948_DMP_Secondary_On_Off_t`, gated by a *different* header field,
  `header2 & DMP_header2_bitmap_Secondary_On_Off`). The plan mentioned
  handling it "the same way dmpStat/FIFO-empty is already handled" for
  the quaternion path - the quaternion path's actual handling of
  missing/empty data is simply "no bit set this frame -> don't update,
  keep last known value," which the compass path already does identically
  by construction (same `haveDmpCompassSample` gate, same "wasn't present
  in this frame" case). Reading and reacting to `Compass_Off` specifically
  would be new behavior (e.g. force-invalidating `haveDmpCompassSample`
  on a real secondary-bus fault) rather than "the same way" - left out to
  keep this change to what the root-cause analysis actually required;
  revisit only if real hardware testing shows stale/stuck compass data
  that the existing gate doesn't catch.
- **No new native test suite** (the plan's `test_dmp_compass` suggestion) -
  unlike the quaternion-rotation fix from the same prior session (which
  added genuine new pure math - `quaternionFor`/`rotateDmpQuaternion` in
  `lib/icm20948pure`, desktop-testable and in fact caught a real bug via
  its own test), this fix has no new pure-math logic. It only changes
  *which hardware-adapter method* feeds the existing, already-tested
  `ImuCoordinateTransform::toBoatFrame()` - the raw FIFO scale-factor
  arithmetic (`/65536.0`) lives in `GwIcm20948HardwareAdapter.cpp`, which
  is Arduino/ICM_20948-library-coupled and outside
  `lib/icm20948pure/icm20948_native_test`'s desktop-testable scope, same
  as `readDmpQuaternion()`'s own `/1073741824.0` scale factor has never
  had a native test either. Correctness of the `2^16` scale factor itself
  still needs the same kind of real-world sanity check the plan asked
  for below, not a native test.
- **Still needed**: a real tumble capture (device physically rotated
  through orientations, not just sitting stationary on a bench) plus the
  ellipsoid-fit quality-gate retest that originally motivated this whole
  investigation - see "Bench verification" below for what WAS checked
  (stationary sanity only).

## Bench verification (2026-08-06, same session as the fix)

Flashed to the real IMU unit over USB (`/dev/cu.usbmodem2101`) and tested
live over its WiFi API (`/api/user/icm20948Task/data`) while connected at
`192.168.8.156`. Two completely separate problems surfaced and were
resolved in the same session - worth keeping distinct since they have
different causes and different fixes:

### Problem 1: DMP wouldn't initialize at all on this bench setup - traced to the power supply, not code

First flash attempt showed `dmpActive:false` and continuous
`Wire.cpp:499 requestFrom(): i2cWriteReadNonStop returned Error -1`
messages (hundreds per 10-second window). Root-caused methodically rather
than guessed at, because the user (correctly) suspected the fix itself
first:

1. **Ruled out the fix, the reorder, and every other uncommitted change
   in the tree** by `git stash`-ing everything back to the last commit
   and reflashing - the identical failure (chip totally undetected on
   3 of 4 resets, `DMP init FAILED` on the 4th) reproduced on code that
   had never been touched this session. This is the single most useful
   verification technique from this session and worth repeating whenever
   a "did my change break this" question comes up with hardware in hand:
   stash, reflash, reproduce-or-not, then pop back - a few minutes of
   build+flash time settles it definitively instead of arguing from
   first principles.
2. **A simple address-level I2C bus scan (already-existing code in
   `GwIcm20948HardwareAdapter::begin()`, just temporarily bumped from
   `GwLog::LOG` to `GwLog::ERROR` to survive this device's log-level
   filter) found the chip reliably on every single reset** - proving
   basic wiring/addressing/pull-ups were fine. It was specifically
   **multi-byte register transfers** (`imu.begin()`'s init sequence, and
   even more so `initializeDMP()`'s ~14KB firmware upload) that failed,
   with a consistent `ICM_20948_Stat_Data_Underflow` status - a
   read-got-fewer-bytes-than-expected signature, not a bus-arbitration
   or address-NACK one.
3. Reseating the Qwiic cable (both ends) made no measurable difference.
   Dropping the I2C clock 400kHz->100kHz measurably reduced the error
   rate in one run but was inconsistent across resets and didn't fix the
   underlying failure - ruled out as a real fix, reverted back to 400kHz.
4. **Actual fix: the user changed the power supply** feeding the board
   (bench USB-only power, not the boat's supply this device normally
   runs on). Immediately after: I2C errors dropped from ~780 per 10s
   window to ~1, `imu.begin()` succeeded on every reset, and
   `initializeDMP()` succeeded every time (`dmpActive:true`, no more
   `Data Underflow`). 4/4 clean resets after the supply change, vs. 0/4
   clean before. **Short address probes tolerate marginal supply noise;
   sustained multi-byte transfers (register writes, firmware upload)
   don't** - worth remembering as a diagnostic pattern: if I2C works at
   the scan/address level but not for actual data transfers, suspect
   power before suspecting the cable or the code, especially on a bench
   setup using different power than the device's normal installed supply.

### Problem 2: the actual magnetometer fix, confirmed against the documented bug signature

With DMP now initializing reliably, polled `/api/user/icm20948Task/data`
repeatedly (stationary on the bench, not a real tumble) and saw exactly
the signature this whole investigation started from, split across the
two fields the fix touches:

- The low-level AGMT register decode, now exposed separately as
  `magAgmtRawX/Y/Z`, sat frozen at **exactly `-38.400`** across 8+
  consecutive polls - the identical 38.4 uT / 256-count-step signature
  documented at the top of this file, reproduced live. Confirms the
  original bug is real and still present in the AGMT shadow-register path,
  exactly as expected.
- `magRawX/Y/Z` was changed after this bench note to mean the effective
  pre-user-calibration magnetometer source feeding the pipeline and CSV.
  With DMP active that is the DMP compass field, not the known-bad AGMT
  decode. `/api/user/icm20948Task/data` reports the selected path in
  `magSource` (`dmp_compass` or `agmt`).
- `magBoatX/Y/Z` (the DMP-corrected values the fix actually wires into
  the pipeline) varied smoothly sample-to-sample (e.g. magBoatX: -31.05,
  -31.95, -33.6, -31.65, -31.2, -30.9, -32.7, -31.8 across 8 polls) -
  full-resolution jitter, not stepped, matching Y/Z's existing healthy
  behavior.
- `magMagnitude` was consistently ~35 uT - a plausible Earth-field
  reading, in the same ballpark as the standalone
  [[icm20948-compass-test]] rig's ~40 uT corrected-magnitude finding
  (different sensor unit and location, so an exact match isn't expected,
  but the order of magnitude and consistency across samples is the real
  sanity check). This is real-world confirmation the `/65536.0` (2^16)
  scale factor is correct, not just plausible-looking source code.
- `headingValid:true`, `headingQuality:"good"`, `fusionValid:true`,
  `magDisturbed:false` all reported healthy with the fix in place.

This is stationary-bench confirmation, not the full tumble-capture
ellipsoid-fit retest - see "Still needed" above for what's still open.
