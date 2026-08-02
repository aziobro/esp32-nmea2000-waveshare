# IMU heading pipeline - release candidate report

**rc/icm20948-hardware-test-1** - not merged to master, not for
production flashing until physical bring-up (see below) confirms it.

## Branch and commit hashes

| | |
|---|---|
| Branch | `feature/icm20948-heading-pipeline` |
| HEAD commit | `027090b4ae0a14aaf277b94d8589a517b0ac4a84` |
| Tag | `rc/icm20948-hardware-test-1` → `027090b` (annotated, pushed) |
| Branch point (merge-base with master) | `dd988bfe7482c12aa8170a669a2b728dea334ce9` |
| `origin/master` (unaffected throughout) | `6f4df3f966c505778198e0205a03f28fcdfb8e9c` |

13 commits since the branch point (12 phase commits + this report):

```
1428cd1 Phase 1: audit implementation against the original plan, wire in gyro correction and rate-of-turn filtering
442da20 Phase 2: runtime CSV diagnostic capture
fc3d7f0 Phase 3: versioned calibration JSON import/export
d359988 Phase 4: offline Python ellipsoid magnetometer calibration tool
e2e2d4f Phase 5a: 2D boat-swing magnetometer calibration engine (ImuMagCal2D)
92df0c9 Phase 5b: wire calibration JSON, gyro cal, and mag cal 2D to HTTP + NVS
44f6019 Phase 5c: calibration/logging web UI, raw magnetic data panel
dd15d41 Phase 6a: extract the per-cycle pipeline into ImuCycleProcessor
4ea1647 Phase 6: debug-only CSV replay tool running the real pipeline
82fe481 Phase 7: performance instrumentation + real-time-loop code review
6b15d8b Phase 8: heading holdover (gyro dead reckoning) + validity audit
4466fe5 Phase 9: verify heading mode semantics, confirm dmp is fully validated
027090b Phase 10: note the shared native-test binary path caveat
```

(This branch already contained 14 earlier commits from the prior
session's request, ending at the branch point `dd988bf` - unchanged by
this work, not relisted here.)

## Files changed (this branch's 13 commits, `dd988bf..HEAD`)

**61 files changed, 7398 insertions(+), 138 deletions(-)**

<details>
<summary>Full list</summary>

New:
- `doc/IcmHeadingModeSemantics.md`, `doc/IcmHeadingValidityAudit.md`, `doc/IcmImplementationAudit.md`, `doc/IcmPerformanceReview.md`
- `lib/icm20948pure/ImuCalibrationJson.{h,cpp}`, `ImuCycleProcessor.{h,cpp}`, `ImuDiagnostics.{h,cpp}`, `ImuHeadingHoldover.{h,cpp}`, `ImuMagCal2D.{h,cpp}`
- `lib/icm20948task/GwIcm20948CalControlTask.{h,cpp}`, `GwIcm20948CaptureTask.{h,cpp}`
- `test/test_calibration_json/`, `test_cycle_processor/`, `test_diagnostics/`, `test_heading_holdover/`, `test_mag_cal_2d/`, `test_replay_tool/`
- `tools/icm20948_calibration/` (full Python package: `calibrate.py`, `csv_loader.py`, `ellipsoid_fit.py`, `quality.py`, `report.py`, `calibration_json.py`, `synthetic.py`, tests, `docs/sample_report/`, requirements files, README)
- `tools/icm20948_replay/README.md`, `tools/icm20948_replay/fixtures/level360Rotation.csv`

Modified:
- `lib/icm20948task/GwIcm20948HardwareAdapter.{h,cpp}` (FIFO frames-drained counter)
- `lib/icm20948task/GwIcm20948Task.cpp` (orchestration rewritten around `ImuCycleProcessor`, new calibration/perf/holdover wiring)
- `lib/icm20948task/icm20948.js` (Raw magnetic data, Calibration, Gyroscope calibration, Diagnostic logging, Performance panels; holdover/disagreement rows)
- `lib/icm20948task/icm20948Config.json` (+3 fields: `icmCalJson`, `icmRotFiltAlpha`, `icmHdgHoldMs`)
- `lib/icm20948task/platformio.ini` (native test env's `lib_deps` gained ArduinoJson)
</details>

## Firmware build results

All 3 real board environments, clean build, this HEAD:

| Environment | Status | RAM | Flash |
|---|---|---|---|
| `waveshare-esp32s3-rs485-can` | SUCCESS | 17.7% (57916 / 327680 B) | 74.1% (1504597 / 2031616 B) |
| `waveshare-esp32s3-rs485-can-garmin` | SUCCESS | 16.7% (54664 / 327680 B) | 67.4% (1369825 / 2031616 B) |
| `waveshare-esp32s3-rs485-can-ais` | SUCCESS | 14.7% (48260 / 327680 B) | 58.2% (1182893 / 2031616 B) |

The `-ais` variant's numbers are unchanged byte-for-byte from before any
of this branch's IMU work: that board variant never calls `initIcm20948`
(no IMU pins defined for it), so the linker dead-strips the entire
icm20948 subsystem including everything added this branch - confirmed,
not assumed, by comparing against a pre-branch build.

Flash headroom: all three boards have 25-42% flash free. RAM headroom is
comfortable (>82% free on all three) - this pipeline is not RAM-bound.

## Native test results

**163/163 passed**, 17 suites, `icm20948_native_test` (desktop, `platform=native`):

```
test_calibration            test_cycle_processor         test_mag_cal_2d
test_fusion                 test_heading_source           test_gyro_cal
test_heading_filter         test_diagnostics               test_simulator_scenarios
test_heading_holdover       test_coordinate_transform      test_angle_math
test_deviation_table        test_quaternion                test_calibration_json
test_mag_monitor            test_compass
test_replay_tool (0 assertions - not a Unity test, see below)
```

## Python test results

**42/42 passed**, `tools/icm20948_calibration/` (`pytest`):
`test_calibration_json.py`, `test_csv_loader.py`, `test_ellipsoid_fit.py`,
`test_quality.py`, `test_synthetic_end_to_end.py`.

## Replay test results

The replay tool (`test/test_replay_tool/test_main.cpp`, built via
`pio test -e icm20948_native_test -f test_replay_tool --without-testing`)
is not a Unity test suite - it's a CLI debug tool, verified instead by
exercising it directly:

- **Fixture self-consistency**: replaying the committed
  `tools/icm20948_replay/fixtures/level360Rotation.csv` (301 samples,
  generated by running the real `ImuCycleProcessor` and recording its own
  output as ground truth) reproduces that exact output: **max heading
  diff 0.00°, mean 0.00°, 0/301 active-source mismatches**.
- **Injection sanity**: `--inject-stale-dmp`, `--inject-mag-disturbance`,
  and `--inject-invalid-quaternion` each independently confirmed to
  change the pipeline's rejection-flags output relative to the
  un-injected baseline at the same sample index (proving the injections
  actually reach the pipeline, not just parse without effect).
- **No-args safety**: running with no CSV argument prints help and exits
  0, confirmed not to fail a full unfiltered `pio test -e
  icm20948_native_test` run.

## Config migration behavior

Zero renames, zero removals. Diffed every `icmXXX` key between `master`
and this branch's HEAD: exactly 3 new additive keys -
`icmCalJson` (text, default `""`), `icmRotFiltAlpha` (number, default
`1.0`), `icmHdgHoldMs` (number, default `5000`). Every pre-existing key
(including the ones renamed during the *prior* session's rewrite -
`icmSrcTransMs`, `icmHdgFiltEn`, `icmHdgFiltTau`, `icmDevEnable` - which
were already on `master` before this branch started) is untouched by
this branch's work.

All 25 `icm20948*` config key names are within the project's 15-character
NVS key limit (checked programmatically; longest is `icmRotFiltAlpha` at
exactly 15).

## Remaining physical-test dependencies

Everything in this report is software-verified only. No physical
ICM-20948 was available this session (stated explicitly at the start of
this work and unchanged throughout). Specifically unverified:

- Real sensor I2C timing (`sensorReadUs`, `totalLoopUs`, whether 10Hz -
  or a higher configured rate - is actually achievable) - see
  `doc/IcmPerformanceReview.md`'s stated prediction, not yet a measurement.
- Real DMP FIFO behavior, actual `fifoFramesDrained`/`fifoOverflows` rates.
- Real magnetometer noise/dropout characteristics - `ImuMagMonitor`'s
  `minMagnitude`/`maxMagnitude`/hysteresis defaults are provisional.
- Whether `icmHdgHoldMs`'s 5000ms default and
  `minConsecutiveSamplesToRecover`'s 3-sample default are well-tuned for
  this specific DMP's real staleness/disagreement behavior.
- The 20 non-flat `MountOrientation` values' real-world meaning (only the
  4 flat orientations are confirmed correct by construction - unchanged
  finding from the prior session).
- Whether the offline Python tool's ellipsoid fit produces a usable
  calibration from an actual hand-tumbled sensor capture (only tested
  against synthetic data - explicitly flagged per this branch's own "do
  not claim hardware accuracy based on synthetic tests" constraint).

## Exact first-device-test sequence

1. Follow `doc/IcmPhysicalTestProcedure.md` steps 1-9 unchanged (build/
   flash, IMU detection, raw-axis test, orientation confirmation, initial
   dockside checks) - nothing in this branch altered that foundation.
2. **New surface check** before calibrating: open the IMU tab and confirm
   all 5 new panels render and update - Raw magnetic data, Calibration,
   Gyroscope calibration, Diagnostic logging, Performance. Confirm
   `perfStatus` numbers look sane (`sensorReadUs`/`processingUs` in the
   tens-to-low-hundreds of microseconds range, `missedDeadlines` staying
   at 0 under normal operation) - this is the first real data point for
   the "is 10Hz achievable" question above.
3. **Calibrate** using ONE of three paths (all new this branch, pick
   whichever fits the situation):
   - Quick/no-tools: IMU tab's Calibration panel → 2D boat swing (Start,
     rotate the boat through a full circle, Stop, review the reported
     coverage/quality, Save if reasonable).
   - Best accuracy: Diagnostic logging panel → Start, tumble the sensor
     by hand through many orientations (ideally off the boat) → Stop →
     Download → run `tools/icm20948_calibration/calibrate.py` on the
     downloaded CSV → Import the resulting `calibration.json` via the
     Calibration panel.
   - Legacy: the original `icmMagXOff`/`icmMagYOff` "C" button flow
     (Config page), unchanged from before this branch.
4. **Gyro calibration**: dockside, boat stationary and level, IMU tab's
   Gyroscope calibration panel → Start → wait for it to complete (motion
   rejects samples without losing progress) → Save.
5. Continue with `doc/IcmPhysicalTestProcedure.md` steps 10-18 unchanged
   (fine offset, interference-source checks).
6. **New**: with `icmHeadingMode` at its default (`dmp`), attempt to
   observe holdover: briefly disrupt DMP (e.g. momentarily block/cover
   the sensor, or introduce a strong magnet near it) and watch the IMU
   tab's "Holdover (gyro-only continuation)" row activate, then confirm
   the last-good PGN 127250 value (if `icmSendHdg` were on) does NOT
   update to a guessed value during that window - it should hold its
   last real transmission or go stale, never send the dead-reckoned
   estimate. **This can only be observed in `dmp` mode** - see
   `doc/IcmHeadingModeSemantics.md`'s finding that holdover isn't
   reachable from `auto`/`software_compass`/`software_9axis_fusion`.
7. Continue with step 19 (trusted-compass comparison) - this remains the
   explicit gate for turning `icmSendHdg` on, unchanged.
8. Steps 20 (deviation table - entries now round-trip through the
   Calibration panel's export/import, still no dedicated point-by-point
   editor, deliberately deferred both sessions) and 21 (PGN verification
   on the Garmin) as originally written.

## Provisional defaults (need physical tuning)

| Config / constant | Default | Where |
|---|---|---|
| `icmRateHz` | 10 Hz | pre-existing |
| `icmSrcTransMs` (source blend) | 1000 ms | pre-existing |
| `icmHdgFiltTau` | 1.0 s | pre-existing |
| `icmRotFiltAlpha` | 1.0 (no smoothing) | this branch, Phase 1 |
| `icmHdgHoldMs` (holdover duration) | 5000 ms | this branch, Phase 8 |
| `DmpValidationConfig` (staleness/jump/disagreement thresholds) | various | pre-existing, unchanged |
| `MagMonitorConfig` (magnitude/change/hysteresis thresholds) | various | pre-existing, unchanged |
| `HeadingHoldoverConfig::minConsecutiveSamplesToRecover` | 3 (fixed, not configurable) | this branch, Phase 8 |
| `HeadingHoldoverConfig::recoveryBlendMs` | 1000 ms (fixed, not configurable) | this branch, Phase 8 |

## Known failure modes / gaps (all deliberately disclosed, not fixed this session)

1. **Compass and fusion modes validate more shallowly than DMP mode** -
   compass never reports invalid (only downgrades quality during
   disturbance), fusion only has a one-time startup gate. Consequence:
   Phase 8's holdover mechanism only engages in `dmp` mode (the default,
   so this is live out of the box, not a hidden limitation) -
   `doc/IcmHeadingModeSemantics.md`.
2. **Holdover state isn't a CSV diagnostic-capture column** - would touch
   3 interdependent, already-tested subsystems for a requirement the web
   UI already satisfies - `doc/IcmHeadingValidityAudit.md`.
3. **`GwConfigHandler` has no internal lock** - a pre-existing,
   project-wide characteristic, made newly relevant by this branch's
   task-triggered config writes (previously only the full-page config
   save flow wrote config, and that always restarts immediately after) -
   `doc/IcmPerformanceReview.md`.
4. **Deviation table has no point-by-point web editor** - entries
   round-trip through calibration JSON export/import only. Deliberately
   not built, per the user's own explicit instruction not to invest in
   it before hardware testing.
5. **20 of 24 `MountOrientation` values are mathematically valid but
   real-world-unconfirmed** - unchanged finding carried over from the
   prior session.
6. Every numeric default in the "provisional defaults" table above is
   untuned against real sensor behavior.

## Rollback instructions

- **This branch was never merged to `master`** - `origin/master` remains
  at `6f4df3f`, untouched and unaffected by any of this work (verified
  via `git ls-remote` at the start and end of this session).
- **To abandon this branch entirely**: nothing to undo on `master`.
  Deleting `feature/icm20948-heading-pipeline` (local and/or
  `origin`) and the `rc/icm20948-hardware-test-1` tag removes all trace;
  this is a purely additive branch.
- **If a physical device is flashed with this RC and needs to be rolled
  back**: reflash from `master`'s own build of the same board
  environment (`git checkout master && pio run -e <env>`) - config
  values this branch newly introduced (`icmCalJson`, `icmRotFiltAlpha`,
  `icmHdgHoldMs`) simply won't exist in `master`'s firmware and will be
  ignored; every pre-existing config key keeps its meaning, so no manual
  NVS cleanup is needed for a downgrade.
- **If only specific new behavior needs disabling without a full
  rollback**: every new feature is independently config-gated and
  defaults to reproducing prior behavior - set `icmHdgHoldMs` to `0` to
  disable holdover entirely, leave `icmCalJson` empty (default) to fall
  back to the legacy hard-iron-only calibration path, `icmSendHdg` stays
  off by default throughout regardless.
