# IMU heading pipeline — implementation audit (Phase 1)

Audit against the original 20-phase request, done before adding more code
per the follow-up instructions. Categories: **Fully implemented**,
**Partially implemented**, **Deferred**, **No longer needed**, **Requires
physical hardware**.

| # | Original phase | Status | Notes |
|---|---|---|---|
| 1 | Refactor into testable components | Fully implemented | `lib/icm20948pure` (13 files), zero Arduino dependency, all under `icm20948_native_test`. |
| 2 | Define coordinate systems | Fully implemented | `ImuCoordinateTransform`, all 24 orientations, 4 flat ones hand-verified, 20 tilted ones mathematically valid but physically unconfirmed (needs hardware). |
| 3 | Complete calibration model | Partially implemented | `ImuCalibration`/`ImuCalibrationOps` fully support bias+matrix for mag, bias+scale for accel, bias for gyro, and legacy migration - tested. JSON import/export does not exist yet (this request's Phase 3). |
| 4 | 2D boat-swing calibration engine | **Deferred - not started** | `ImuMagCal2D` was planned but never created. Only the *result* it would produce (`ImuCalibration`) exists. Needed for this request's Phase 5 web panel. |
| 5 | Offline full 3D calibration (CSV + Python tool) | Deferred | Neither the CSV capture format nor the Python tool exist yet (this request's Phases 2 and 4). |
| 6 | Calibrated tilt-compensated heading | Fully implemented | `ImuCompass`, verified against synthetic ground truth across headings/roll/pitch/hard-iron/soft-iron. |
| 7 | Sensor simulator | Fully implemented | `ImuSimulator`, all 12 scripted scenarios. |
| 8 | Quaternion handling + DMP validation | Fully implemented | `ImuQuaternion` + `DmpValidator`, `HeadingRejectReason` bitmask. |
| 9 | Selectable heading sources | Fully implemented | `HeadingSourceSelector`, all 5 modes, auto policy, circular blend-on-transition. |
| 10 | Magnetic-field monitoring | Fully implemented | `ImuMagMonitor`, hysteresis. |
| 11 | Heading filtering | Fully implemented | `ImuHeadingFilter`, circular, motion-adaptive. Wired into the task but gated behind `icmHdgFiltEn` (default off) - see wiring audit below. |
| 12 | Rate of turn improvements + gyro calibration | Partially implemented | `GyroCalEngine` and `ImuRateOfTurn` exist and are tested, but **neither was called from `GwIcm20948Task.cpp`** until this commit (see fixes below) - ROT was a raw, unfiltered gyro-Z reading with no bias correction and no plausibility cross-check. |
| 13 | Synchronize NMEA2000 output | Fully implemented | One cycle, one set of candidates, one selection, PGN 127257/127250/127251 emitted from the same result. |
| 14 | Deviation table | Partially implemented | `ImuDeviationTable` fully implemented/tested; wired into the task behind `icmDevEnable` (default off), but there is no way to populate it yet (empty table = no-op even when enabled). Web editor deferred until after hardware testing per this request's explicit instruction. |
| 15 | Web interface | Partially implemented | IMU tab shows live values, source diagnostics, rejection reasons. No calibration/logging/gyro-cal controls yet (this request's Phase 5). |
| 16 | Debug replay mode | Deferred | Not started (this request's Phase 6). |
| 17 | Automated tests | Fully implemented | 109 host-side tests, real numeric tolerances, 13 suites. |
| 18 | Config migration + safe defaults | Fully implemented | Every legacy key unchanged; `migrateFromLegacy` verified to reproduce the old formula exactly; `icmSendHdg` still off by default. |
| 19 | Physical test procedure | Fully implemented (as a document) | `doc/IcmPhysicalTestProcedure.md` - the procedure itself obviously requires hardware to *execute*. |
| 20 | Hardware test worksheet | Fully implemented | `doc/IcmHardwareTestWorksheet.{md,csv}`. |

**No longer needed**: nothing identified - every deferred item is still
wanted, just not yet built.

**Requires physical hardware** (cannot be closed by more code, listed
here so they don't get chased further in software): confirming which of
the 20 non-flat `icmOrientation` values matches which real mounting;
tuning every "provisional" threshold (DMP validation, mag monitor
limits, filter time constants, gyro-cal gates); the Mahony filter's
`kp`/`ki` gains against real sensor noise; anything in
`doc/IcmPhysicalTestProcedure.md`.

## Runtime wiring verification

Checked by direct code inspection of `GwIcm20948Task.cpp`, not assumed:

| Pure module | Called from the task? | Detail |
|---|---|---|
| Coordinate transform | Yes | `toBoatFrame` on accel, gyro, mag every cycle. |
| Magnetometer calibration | Yes | `applyMag` every cycle - but always with an identity soft-iron matrix (`migrateFromLegacy` never produces a real one yet), so the matrix code path itself is untested at runtime, only bias-subtraction is. |
| Tilt-compensated heading | Yes | `ImuCompass::rawHeadingDeg` every cycle, unconditionally (independent of active mode, matching the design goal). |
| Mahony fusion | Yes | `fusion.update()` every cycle, unconditionally (always warm, ready if the mode is switched to it). |
| DMP validation | Yes | `dmpValidator.validate()` every cycle DMP is enabled, regardless of `icmHeadingMode` - confirms Phase 9's requirement (validation is active in `dmp` mode, not bypassed) was already true before this request. |
| Heading-source selection | Yes | `sourceSelector.update()` every cycle. |
| Circular filtering | Conditionally | Only when `icmHdgFiltEn` is on (default off, to preserve pre-rewrite behavior exactly). |
| Magnetic monitoring | Yes | `magMonitor.update()` every cycle; its `Disturbed` state feeds DMP/compass candidate quality. |
| Gyro correction | **No - fixed in this commit** | `ImuCalibrationOps::applyGyro()` was never called; gyro bias (always 0 today, since nothing populates it) was silently not subtracted. Now applied before fusion/ROT. |
| Rate of turn | **No - fixed in this commit** | `ImuRateOfTurn::lowPass`/`derivedFromHeadingDegPerSec`/`disagreesWithHeadingDerivative` were never called; ROT was raw, unfiltered gyro-Z. Now low-pass filtered (alpha defaults to 1.0 = unfiltered, so output is unchanged unless `icmRotFiltAlpha` is explicitly lowered) and cross-checked against the output heading's derivative for diagnostics. |
| Deviation correction | Conditionally | Only when `icmDevEnable` is on (default off) - and even then, currently always a no-op (empty table, no editor yet). |
| NMEA solution creation | Yes | PGN 127257/127250/127251 all emitted from the same cycle's `HeadingSourceSelector::Result`. |

## Dead code found and resolved

- `ImuCalibrationOps::applyGyro()` - unused, now called (see above). Not
  dead code to remove; a real gap, fixed.
- `ImuRateOfTurn::*` - unused, now called (see above). Fixed, not removed.
- `GyroCalEngine` - genuinely still unused after this commit. **Left
  in place, not removed**: it's fully implemented and tested, and
  wiring it in requires an interactive start/stop/save workflow that
  belongs with this request's Phase 5 (web interface), not a bare
  function call - there's nothing meaningful to "use" yet without a
  trigger and a place to persist the result. Will be wired in Phase 5.
- No other unused pure-math classes found.
