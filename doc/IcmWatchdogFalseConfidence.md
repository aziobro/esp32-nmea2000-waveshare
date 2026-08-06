# I2C hang watchdog - false confidence, root cause, and fix

Found and fixed live on the boat-unit's bench (2026-08-06), while trying to
bench-verify the magnetometer/DMP fix (`doc/IcmMagnetometerDmpConflict.md`)
via physical tumbling. Physically handling the sensor (flexing the Qwiic
cable) reliably wedges its I2C bus - a previously-documented, previously
"fixed" issue (see the "I2C hang watchdog" entry in
`doc/IcmMagnetometerDmpConflict.md`'s "Other loose ends" section, which
turned out to be **wrong** - the watchdog had never actually been proven
to recover a real hang, only assumed to from the surrounding circumstances
of an earlier session).

## Symptom

During bench tumbling, `roll`/`pitch`/`heading` on
`/api/user/icm20948Task/data` would freeze at bit-identical values
indefinitely (confirmed frozen for 90+ seconds straight in one case) while
the rest of the device (WiFi, HTTP, other tasks) kept responding normally.
The existing software watchdog (`icm20948WatchdogTaskEntry` in
`GwIcm20948Task.cpp`, an 8-second heartbeat-timeout mechanism intended to
force a self-recovering reboot) never fired. Recovery required a manual
hardware reset (USB DTR/RTS toggle, or physically power-cycling on the
boat) every time - exactly the failure mode the watchdog was supposedly
already built and confirmed to prevent.

## Investigation - three plausible fixes in a row, all wrong

Each of these was a real, defensible issue found by reading the code, and
each was verified NOT to be sufficient by actually reproducing the hang
afterward - worth recording precisely because "this is obviously the bug"
was wrong three times before finding the real one, and each wrong theory
still left behind a legitimate, kept improvement:

1. **Core starvation theory**: all user tasks (including the watchdog)
   were created via the same unpinned `xTaskCreate(...)` call at equal
   priority - if the I2C hang were a hardware-register busy-spin that
   never yields, it could starve anything else scheduled on the same
   core, watchdog included, regardless of priority. **Fix applied**:
   extended `GwApi::addUserTask()`/`GwUserCode` with an optional
   `coreId` parameter (default -1 = `tskNO_AFFINITY`, zero behavior
   change for every other caller), pinned the watchdog to core 0 and the
   sensor task to core 1 via `xTaskCreatePinnedToCore`. **Confirmed via
   direct core-ID logging that the pinning took effect exactly as
   intended** - and the hang still wasn't recovered. Kept anyway (real,
   harmless robustness improvement) but wasn't the actual bug.
2. **Cross-core visibility theory**: the heartbeat/armed flags were
   plain `volatile unsigned long`/`volatile bool` globals shared between
   the two now-confirmed-different cores - `volatile` blocks compiler
   reordering but is not a cross-core memory barrier, so a write on one
   core isn't guaranteed to become visible to the other in any bounded
   time. **Fix applied**: switched both to `std::atomic` (default
   sequentially-consistent ops, which do emit real memory-barrier
   instructions on this dual-core Xtensa target). **Still didn't
   recover the hang** on the next real repro. Kept anyway (genuinely
   more correct than the plain-volatile version), but also not the bug.
3. **Log-mutex deadlock theory**: `GwLog::logDebug()`/`GwLog::flush()`
   both call `xSemaphoreTake(locker, portMAX_DELAY)` on a single GLOBAL
   mutex shared by every `LOG_DEBUG` call in the entire firmware, with
   an infinite timeout. If the hung sensor task had been stuck mid-
   `LOG_DEBUG` while holding that lock, the watchdog's own `LOG_DEBUG`
   call in its restart branch would block forever on the same mutex,
   and the restart would never happen. **Fix applied**: replaced the
   watchdog's `LOG_DEBUG`/`logger->flush()` in the restart path with a
   direct `USBSerial.printf()` + `USBSerial.flush()`, bypassing
   `GwLog`'s mutex entirely for this one critical path. **Still didn't
   recover the hang.** Kept anyway (a real deadlock risk regardless of
   whether it was THIS bug), but also not the actual root cause.

## Actual root cause

None of the above, because none of them addressed what was actually
happening: **the sensor task was never blocked or starved at all.**
Added a real-time, unconditional diagnostic (the watchdog printing
heartbeat age + a lightweight "last checkpoint" marker every 2 seconds,
regardless of whether a restart was imminent) and watched it live during
a real hang:

```
heartbeat age=12 ms, checkpoint=before dataReady
heartbeat age=98 ms, checkpoint=before dataReady
heartbeat age=83 ms, checkpoint=before dataReady
... (repeats for 90+ seconds, age never climbing, checkpoint never advancing)
```

The main loop (`runIcm20948Task()`'s `while(true)`) was refreshing
`g_icm20948LastHeartbeatMs` **unconditionally at the very top of every
iteration**, before even checking `hw.dataReady()`:

```cpp
while (true)
{
    g_icm20948LastHeartbeatMs = millis();  // <- refreshed regardless
    delay(loopDelayMs);
    if (!hw.dataReady())
        continue;                          // <- loops back up, refreshes again
    ...
```

Once the physical I2C fault got the ICM-20948 into a state where
`hw.dataReady()` (a lightweight status-register check) started returning
`false` forever - not hanging, just perpetually "no new data" - the loop
spins harmlessly through `delay()` + `continue` indefinitely. Every
single iteration refreshes a heartbeat that therefore looks perfectly
fresh (never older than one loop period), even though zero real sensor
work has happened and `roll`/`pitch`/`heading` never update again. A
heartbeat-based watchdog **cannot** distinguish "task genuinely blocked"
from "task alive and looping but making zero forward progress" unless the
heartbeat itself is defined as "last time real progress happened" - which
this one wasn't.

## Fix

Moved the heartbeat refresh to occur only on a cycle that actually got
past the `dataReady()` gate:

```cpp
while (true)
{
    g_icm20948LastCheckpoint = "before dataReady";
    delay(loopDelayMs);
    if (!hw.dataReady())
        continue;
    g_icm20948LastHeartbeatMs = millis();  // only now - genuine progress
    g_icm20948LastCheckpoint = "after dataReady, before readAGMT";
    ...
```

At the configured rate (10Hz+), `dataReady()` returning true well within
the 8-second timeout is the normal case, so this costs no real margin
against the original hang scenario the watchdog was built for either -
it only removes the specific blind spot that let a "spinning but not
blocked" fault look identical to healthy operation.

## Verification

Bench-confirmed on the real IMU unit, same session: with the fix in
place, physical tumbling triggered the underlying I2C fault three times
in a single 90-second window (the fault itself is unfixed - a real,
separate hardware-level issue - and can recur at any time handling this
sensor) - and every single time, the watchdog correctly detected the
stale heartbeat, successfully logged the restart reason (confirming the
log-mutex bypass fix was also necessary, not just theoretically prudent -
the message never printed before that fix), and the device rebooted
cleanly and came back up healthy (`dmpActive`/`headingValid` both true
again) within about 20 seconds each time. Re-confirmed once more after
removing the temporary diagnostic-only logging (per-2-second heartbeat
print, one-time core-ID confirmation, per-task-creation ERROR bump) added
during the investigation - the real fixes (heartbeat-on-progress, direct
serial write in the restart path, core pinning, atomics) all stayed; only
the noisy always-on diagnostics were removed.

## What's still open

- **The underlying I2C fault itself is not fixed** - only converted from
  "permanent hang requiring a manual reset" into "automatic ~20s
  self-recovering reboot." It's still triggered readily by physically
  handling/flexing the Qwiic connection during bench tumbling. Once the
  unit is actually boat-mounted (not hand-held), this may be far less
  frequent - genuinely unknown until tested in that condition.
- Root cause of the I2C fault itself (why `dataReady()` specifically
  gets stuck returning false, as opposed to the chip going fully
  unresponsive) not investigated - would need a logic analyzer on the
  actual SDA/SCL lines during a live occurrence to say more.
- `GwLog::logDebug()`/`flush()`'s single global infinite-timeout mutex is
  a latent deadlock risk for ANY task on ANY board in this project, not
  just this one watchdog path - worth a proper fix (e.g. a bounded
  timeout, or per-severity lock-free fast path for ERROR-level messages)
  in a dedicated session, not patched piecemeal per call site.
