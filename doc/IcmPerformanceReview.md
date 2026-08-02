# icm20948 performance instrumentation and real-time-loop code review

Phase 7 of the post-Request-B work: lightweight performance counters
(surfaced on the IMU web tab's new Performance panel, `GET
/api/user/icm20948Task/perfStatus`) plus a code review of
`GwIcm20948Task.cpp`'s hot loop and its supporting classes for the
specific hazards the user asked about. This documents findings, not just
"everything is fine" - some items below were fixed, some are flagged and
left as documented, accepted tradeoffs (with the reasoning for that
judgment call), consistent with how `doc/IcmImplementationAudit.md`
handled Phase 1's findings.

## Intended processing frequency and whether it's achievable

`icmRateHz` defaults to 10 Hz (100 ms budget/cycle), configurable 1-50 Hz
(25 ms budget at the top end). The loop's own `delay(loopDelayMs)` paces
polling; work done inside a cycle (sensor read, pipeline processing, NMEA
sends, diagnostic logging) must fit inside that budget or cycles start
running long (see "missed deadlines" below).

No hardware access this session to get real timing numbers, so this
can't yet be answered with measured data - that's exactly what the new
Performance panel is for on first physical bring-up. What can be said
from the code alone: every per-cycle computation is either fixed-size
arithmetic (coordinate transforms, quaternion math, the Mahony fusion
filter, heading-source selection) or bounded by small compile-time
constants (deviation table: 36 entries max; capture queue: 64 entries
max) - nothing in the pipeline itself scales with runtime state in a way
that could grow unboundedly slower over time. At 10 Hz, the ESP32-S3
(240 MHz, hardware FPU) should have a very comfortable margin for this
amount of trig-heavy but otherwise simple floating-point work; the more
likely constraint is I2C transaction time (`hw.readAGMT()`,
`hw.readDmpQuaternion()`), which the Performance panel's `sensorReadUs`
number will make visible on first device test. This is a stated
prediction, not a hardware-verified conclusion - do not treat it as
confirmed until the panel has real numbers.

## Performance panel fields

All timing values are microseconds, "last cycle / running max since
boot." Instrumentation itself is always-on (not a config toggle) - the
overhead is a handful of `micros()` calls and two FreeRTOS heap/stack
queries per cycle, no allocation, negligible next to what it measures.

| Field | What it measures |
|---|---|
| `sensorReadUs` | `hw.readAGMT()` + boat-frame coordinate transforms + `hw.readDmpQuaternion()`, summed |
| `processingUs` | The entire `ImuCycleProcessor::process()` call (includes fusion) |
| `fusionUs` | Just the `MahonyFusion::update()` call inside processing, timed with `std::chrono` inside the pure module itself (portable, no Arduino dependency - see `ImuCycleProcessor.cpp`) |
| `nmeaSendUs` | Sum of all `SetN2k*`/`api->sendN2kMessage()` calls this cycle (up to 3: rate of turn, attitude, heading) |
| `loggingEnqueueUs` | The `capture.offerSample()` call (Phase 2 diagnostic CSV capture) |
| `totalLoopUs` | Full cycle duration, from just after `dataReady()` confirms fresh data through the end of the loop body (excludes the deliberate `delay()` pacing) |
| `missedDeadlines` | Count of cycles where `totalLoopUs` exceeded the configured update interval |
| `fifoFramesDrained` / `fifoOverflows` | From `GwIcm20948HardwareAdapter`: frames actually pulled from the DMP FIFO, and read-status errors, both since boot |
| `freeHeapBytes` / `minFreeHeapEverBytes` | `xPortGetFreeHeapSize()` / `xPortGetMinimumEverFreeHeapSize()` - global heap, not IMU-task-specific, but this is the only task doing any allocation-adjacent work on this path |
| `stackHighWaterMarkBytes` | `uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)` for the IMU task itself - lowest-ever remaining headroom against its 16000-byte allocation |

## Code review findings

### Dynamic allocation in the sensor loop

**Finding, fixed:** none found using `new`/`malloc` in the per-cycle hot
path. `Icm20948Capture::startCapture()` does `malloc()` a buffer, but only
from an HTTP-triggered action (webserver thread), never from the sensor
loop itself.

**Finding, documented (not fixed - pre-existing project-wide pattern):**
Arduino `String` construction happens every cycle in a few places, each
triggering a small heap allocation:
- `headingModeStr` (config read every cycle to parse `icmHeadingMode`) -
  pre-existing since the original heading-source-mode rewrite, not
  introduced this session.
- `calJsonStr` (config read every cycle to detect calibration changes -
  introduced in Phase 5).
- `Icm20948WebData::headingSource`/`headingQuality` `String` members,
  reassigned from a `const char*` every cycle in `setDiagnostics()`.

All three are bounded (config values are at most a few hundred bytes;
`headingSource`/`headingQuality` are short fixed strings) and this
pattern already existed for several other `icmXXX` config fields before
this session touched the file. Fixing it properly would mean changing
`GwConfigHandler`'s read API project-wide (it has no "changed since last
read" query, no non-`String` accessor for text fields) - out of scope for
this task. Flagged here rather than silently left; if `sensorReadUs`/
`totalLoopUs` numbers from real hardware ever show this mattering, revisit.

### Unbounded containers

**Finding: none.** Every container in the hot path has a fixed compile-
time capacity: `DeviationTable` (`MAX_ENTRIES = 36`, plain array, no
dynamic growth), `ImuMagCal2D::sectorVisited` (`MAX_SECTORS = 36`, plain
array), `Icm20948Capture`'s sample queue (`QUEUE_CAPACITY = 64`,
FreeRTOS queue) and RAM buffer (bounded by the configured `maxKB`,
checked before every write, capture stops rather than overflows - see
`Icm20948Capture::appendToBuffer`).

### Blocking locks / storage writes

**Finding, reviewed and accepted:** `calControl.feedGyroSample()` and
`feedMagSample()` take a mutex every cycle (`GWSYNCHRONIZED(lock)` in
`Icm20948CalControl`) to guard the interactive calibration engines
against the webserver thread's start/stop/save actions. Worst-case hold
time on the *other* side (the HTTP handler) is a few statement
executions, never I/O - bounded and short. `GwConfigHandler::updateValue()`
(synchronous NVS flash write) is only ever called from
`Icm20948CalControl`'s HTTP handlers (webserver thread) for
save/import/reset actions - confirmed never called from the sensor loop.
`Icm20948Capture::offerSample()` is lock-free by design (non-blocking
`xQueueSend` with a 0 timeout, drops rather than blocks when full).

### Large stack objects

**Measured** (`sizeof`, native build): `ImuCycleInput` 1024 bytes
(dominated by an embedded `ImuCalibration` at 208 bytes and `DeviationTable`
at 592 bytes), `ImuCycleOutput` 192 bytes, `DiagnosticSample` 296 bytes.
Peak simultaneous stack use from these locals in one cycle is roughly
1.5 KB against the task's 16000-byte stack (~9%) - not concerning, but
worth having the real number on record rather than assuming.
`ImuCycleProcessor` itself (272 bytes: the fusion filter, DMP validator,
source selector, mag monitor, heading filter, and unwrap accumulator's
combined state) is NOT a per-cycle stack object - it's a single instance
living for the task's entire lifetime, declared once outside the loop.

### Accidental large-struct copies

**Finding, fixed:** the calibration-caching block computed
`ImuCalibration cal = cachedCal;` (a 208-byte copy) and then separately
assigned `cycleIn.cal = cal;` (a second 208-byte copy) a few dozen lines
later. `ImuCalibrationOps::applyGyro` already takes its calibration
argument by `const&`, so the intermediate `cal` local was unnecessary -
removed; both use sites now reference `cachedCal` directly, halving this
particular per-cycle copy cost.

**Reviewed, no further findings:** `cycleIn.deviationTable = cachedDeviationTable;`
(up to 592 bytes) is a single, necessary copy - `ImuCycleInput` needs its
own value semantics as an isolated per-cycle snapshot passed into
`ImuCycleProcessor::process(const ImuCycleInput&)` (by reference, so no
*additional* copy happens on the call itself).

### Excessive float conversions

**Reviewed, no findings requiring a fix.** Degree/radian conversions
happen at essentially every stage boundary (`ImuCycleProcessor.cpp`'s
local `toDeg`/`toRad` helpers), which is inherent to mixing a
degrees-based external API/config surface with radians-based trig calls
- not "excessive" in the sense of redundant round-tripping, each
conversion serves a real boundary crossing. No stage converts back and
forth needlessly.

### Data races between web/logging/IMU tasks

**Reviewed:** `Icm20948WebData` and `Icm20948PerfStats` are both
mutex-guarded (`GWSYNCHRONIZED`), matching the established pattern this
file already used before this session. `Icm20948Capture`'s cross-thread
state (stats counters, control flags) uses plain `volatile` fields with
no lock - written from one side, read from the other, accepted as
already-reviewed/pre-existing (Phase 2). `Icm20948CalControl`'s
`GyroCalEngine`/`ImuMagCal2D` engines are locked; `GwConfigHandler` itself
has no internal lock (a project-wide, pre-existing characteristic - see
the comment on `Icm20948CalControl`'s class doc in
`GwIcm20948CalControlTask.h`), and this task's cycle reads several config
values that only the webserver thread's calibration actions write
(`icmCalJson`, `icmOrientation`, `icmDevEnable`) - low risk in practice
since these are simple scalar/string reads, not multi-field invariants
that need to be observed atomically together, but a genuinely new
cross-thread write path introduced this session (previously, only the
project's standard config-save flow wrote config values, and that always
restarts the device immediately after). Flagged for awareness; fixing it
would mean adding locking to `GwConfigHandler` project-wide, out of scope
here.
