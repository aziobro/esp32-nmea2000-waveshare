# ICM-20948 physical test procedure

Run this once the device is physically available again. Nothing here can
be verified without hardware - see `doc/IcmHeadingArchitecture.md` for
what's already been verified in software (109 host-side tests, all three
board environments building clean).

Use `doc/IcmHardwareTestWorksheet.md` (or the `.csv` version) to record
results as you go.

Throughout: `icmSendHdg` should stay OFF until step 19 confirms the
heading is trustworthy - an uncalibrated/unverified heading on the bus is
worse than none.

## 1. Build and flash verification

- **Control**: `pio run -e waveshare-esp32s3-rs485-can` (or whichever
  board env has the IMU), then flash via the established method for this
  board (`esptool.py --no-stub`, see `doc/WaveshareRs485Can.md`).
- **Record**: build succeeds, firmware version string on the Status page.
- **Expected**: clean build (already confirmed pre-flash), device boots,
  web UI reachable.
- **Pass/fail**: boots without a crash loop, web UI's Status page loads.
- **Likely failure cause**: a build config that only manifests on real
  hardware (unlikely given all three board envs already build clean) or
  a flashing procedure issue unrelated to this change.
- **Next action**: if it boots but crashes, capture the serial log at
  `GwLog::ERROR` level and check whether it's IMU-task-related or
  unrelated.

## 2. IMU detection

- **Control**: power on, open the IMU tab.
- **Record**: "Sensor fusion (DMP)" row value, whether the tab shows any
  data at all within a few seconds.
- **Expected**: `dmpActive` reflects whether `icmUseDmp` succeeded; if
  the sensor isn't found at all, the task logs `ICM20948 not found` and
  stops (same behavior as before - unchanged).
- **Pass/fail**: sensor found, roll/pitch/accel values appear and update.
- **Likely failure cause**: I2C wiring, wrong SDA/SCL pin assumption for
  this specific physical unit (see `GWICM20948_SDA_PIN`/
  `GWICM20948_SCL_PIN` in `GwWaveshare485CanTask.h`).
- **Next action**: swap SDA/SCL if reversed (zero-risk, documented
  pattern already used elsewhere in this project).

## 3. Raw-axis movement test

- **Control**: gently move/rotate the sensor by hand (bench test, before
  final mounting) while watching "Acceleration X/Y/Z" on the IMU tab.
- **Record**: which axis changes for which physical motion.
- **Expected**: exactly one axis should read ~1g when that axis points
  straight down, ~0g when horizontal.
- **Pass/fail**: axes respond to motion in a way consistent with SOME
  MountOrientation (doesn't need to match Forward yet).
- **Likely failure cause**: N/A at this stage - purely observational.
- **Next action**: note which physical axis is which for step 4.

## 4. Board-orientation confirmation

- **Control**: compare step 3's observations against the intended
  physical mounting (which way the bow-marked edge points).
- **Record**: which `icmOrientation` value's description best matches.
- **Expected**: `Forward` if mounted flat with the marked edge pointing
  at the bow; otherwise pick the closest match from the 24 (see
  `doc/IcmHeadingArchitecture.md`'s caveat about the 20 unconfirmed
  tilted/upside-down entries).
- **Pass/fail**: N/A - informational, sets the starting point for steps
  5-7.
- **Next action**: set `icmOrientation`, save (restarts the device),
  proceed to step 5.

## 5. Roll sign test

- **Control**: heel the boat (or physically tilt the bench-mounted
  sensor) a known way to starboard.
- **Record**: "Roll (heel), calibrated" value and sign.
- **Expected**: positive while heeling to starboard (NMEA2000
  convention).
- **Pass/fail**: sign matches.
- **Likely failure cause**: wrong `icmOrientation`, or a mount that's a
  180-degree flip of the assumed one.
- **Next action**: if sign is backwards, try `icmRollInv` first (a pure
  sign flip); if magnitude/behavior also looks wrong (not just sign),
  reconsider `icmOrientation` instead - see the Config page's own
  description of Invert vs the fine offset for why these are different
  corrections.

## 6. Pitch sign test

- **Control**: lift the bow.
- **Record**: "Pitch (trim), calibrated" value and sign.
- **Expected**: positive while the bow rises.
- **Pass/fail**: sign matches.
- **Likely failure cause / next action**: same as step 5, using
  `icmPitchInv`.

## 7. Starboard-turn heading sign test

- **Control**: with `icmHeadingMode` set to `software_compass` (isolates
  the compass candidate specifically), turn the bow to starboard.
- **Record**: "Software compass candidate heading" direction of change.
- **Expected**: increases while turning to starboard.
- **Pass/fail**: direction matches.
- **Likely failure cause**: chip axis handedness (unknowable from code -
  documented in `icmHdgInv`'s own description).
- **Next action**: toggle `icmHdgInv` if backwards.

## 8. Stationary gyro calibration

- **Control**: `GyroCalEngine` is implemented and tested but has no web
  UI trigger yet (see `doc/IcmHeadingArchitecture.md`'s deferred list) -
  this step is currently a placeholder pending that UI work.
- **Record**: N/A until the UI exists.
- **Next action**: wire up a start/status control on the IMU tab, then
  run this step (boat stationary, engine off, dead calm or on a mooring)
  before proceeding further.

## 9. Full 3D magnetometer capture before installation, if practical

- **Control**: the offline Python calibration tool and CSV capture
  logging described in the original spec are not yet built (see
  deferred list). If practical, physically rotate the sensor through as
  many orientations as possible by hand before final mounting, recording
  raw magnetometer readings via serial log (`LOG_DEBUG` at a temporarily
  raised level) as a manual substitute.
- **Next action**: build the CSV capture endpoint and offline tool
  (deferred work) before attempting a real 3D fit.

## 10. Installed 360° boat swing

- **Control**: with the sensor in its final mounted position, slowly
  rotate the whole boat through at least one full circle (engine idle
  slow ahead in neutral water, or warp around a mooring) while watching
  the "C" button preview for `icmMagXOff`/`icmMagYOff` stabilize.
- **Record**: final X/Y offset values, whether the preview visibly
  stabilized (stopped changing) by the end of the circle.
- **Expected**: values converge and stop changing.
- **Pass/fail**: stable by the end of one full circle.
- **Likely failure cause**: swing too fast, or a moving disturbance
  nearby (another vessel, dock steel) during the swing.
- **Next action**: redo the swing slower/further from disturbances if
  it didn't stabilize.

## 11. Tilt test

- **Control**: with a known heading held, heel the boat (or, at the
  dock, rock it) and watch "Heading (output)".
- **Record**: heading value across a range of heel angles.
- **Expected**: heading stays constant regardless of heel (this is the
  entire point of tilt compensation - verified mathematically already,
  see `test_compass`'s tilt-compensation tests, but confirms it holds
  with a REAL sensor/real noise too).
- **Pass/fail**: heading varies by less than a few degrees across normal
  heel angles.
- **Likely failure cause**: roll/pitch sign wrong (steps 5-6 not yet
  correct), or `icmOrientation` wrong.
- **Next action**: revisit steps 4-6 before trusting this test.

## 12. Engine-off comparison

- **Control**: engine off, record heading and magnetic field magnitude
  (IMU tab).
- **Record**: heading, "Magnetic field magnitude", "Magnetic disturbance
  state".
- **Expected**: "normal" disturbance state.
- **Baseline for steps 13-18.**

## 13. Engine-running comparison

- **Control**: start the engine, same position/heading as step 12.
- **Record**: same fields, compare against step 12's baseline.
- **Pass/fail**: field magnitude/heading change is small; disturbance
  state ideally stays "normal" (a large alternator/starter motor nearby
  is a plausible real disturbance source - this test is specifically
  designed to catch that).
- **Next action**: if disturbed, consider relocating the sensor further
  from the engine/alternator, or accept and note it as an operational
  limitation (heading briefly unreliable at engine start).

## 14. Alternator-load comparison

- **Control**: with engine running, cycle a large electrical load
  (windlass, high-output charging) on and off.
- **Record**: same fields as step 12, during load vs not.
- **Pass/fail/next action**: same reasoning as step 13.

## 15. Autopilot operation

- **Control**: engage the autopilot (if it has its own motor/clutch near
  the sensor) and observe.
- **Record/pass-fail/next action**: same pattern as step 13.

## 16. Bilge pump operation

- **Control**: run the bilge pump.
- **Record/pass-fail/next action**: same pattern.

## 17. VHF transmission

- **Control**: key the VHF (high power) near the installation.
- **Record/pass-fail/next action**: same pattern - RF interference on
  the I2C bus is a plausible failure mode distinct from magnetic
  disturbance (could show as a sensor read error rather than a mag
  disturbance flag).

## 18. Charger and inverter operation

- **Control**: run the shore power charger and/or inverter.
- **Record/pass-fail/next action**: same pattern.

## 19. Comparison with a trusted magnetic compass

- **Control**: compare "Heading (output)" against a known-good steering
  compass across several actual boat headings.
- **Record**: both readings at each heading, the difference.
- **Expected**: within a few degrees (hard-iron-only calibration,
  documented residual error).
- **Pass/fail**: this is the gate for turning `icmSendHdg` on.
- **Likely failure cause**: incomplete calibration (redo step 10), or a
  disturbance source identified in steps 13-18 that's active during
  normal operation.
- **Next action**: if consistently within a few degrees, safe to enable
  `icmSendHdg`. If there's a heading-dependent pattern to the error
  (not just a constant offset), that's exactly what step 20's deviation
  table is for.

## 20. Optional deviation-table creation

- **Control**: `ImuDeviationTable` is implemented and tested, but has no
  web editor yet (deferred - see `doc/IcmHeadingArchitecture.md`).
- **Next action**: build the web editor, then record measured-vs-
  reference heading pairs at 12+ points around the compass (matching
  step 19's comparison data) and enable `icmDevEnable`.

## 21. NMEA2000 PGN verification on the Garmin

- **Control**: with `icmSendHdg` on, check the Garmin bridge's own NMEA
  2000/0183 diagnostics for PGN 127250 / `$--HDG`/`$--HDT`, matching the
  process already used earlier this project for verifying PGN 127257
  Attitude reached the chartplotter correctly.
- **Record**: sentence/PGN presence, value, update rate.
- **Pass/fail**: heading appears and matches the IMU tab's own value.
- **Likely failure cause**: if attitude (127257) already works but
  heading (127250) doesn't, check `icmSendHdg` is actually on and that
  `icmHeadingMode`'s selected source is currently valid (IMU tab's
  "Active source"/"Quality" rows).
