# Heading validity and holdover audit

Phase 8 of the post-Request-B work: audits heading validity/holdover
behavior against the user's checklist, item by item. Before this phase,
**no holdover mechanism existed at all** - `HeadingSourceSelector`
reported `valid=false` the instant every candidate source dropped out
(confirmed by reading `ImuHeadingSource.cpp` directly). This phase adds
one: a new pure module, `ImuHeadingHoldover`
(`lib/icm20948pure/ImuHeadingHoldover.h/.cpp`), sitting between the
selector and the rest of `ImuCycleProcessor`'s heading-finalization
stage, plus a new `icmHdgHoldMs` config field (default 5000ms, 0 disables
holdover entirely).

## Checklist

**Invalid heading isn't sent as valid PGN 127250.**
Confirmed and enforced at two levels: (1) PGN 127250 is only sent when
`cycleOut.headingValid` is true (unchanged, pre-existing gate); (2) newly
added this phase - a holdover-derived (gyro-only) heading is `valid=true`
for on-screen/logged diagnostics but is explicitly excluded from
transmission via `!cycleOut.headingHoldover` in the send gate
(`GwIcm20948Task.cpp`). A time-limited dead-reckoning estimate is not a
sensor reading, and transmitting one to an autopilot-facing PGN is
exactly the kind of risk this project has stayed conservative about
throughout - see `ImuHeadingHoldover.h`'s class comment.

**Roll/pitch continue independently.**
Confirmed by design and by test
(`test_roll_pitch_continue_independently_of_heading_holdover`,
`test/test_cycle_processor/test_main.cpp`): `ImuCycleOutput::attitudeValid`
is computed entirely from `haveAttitudeSample` (DMP-euler or accel-tilt),
which has no dependency anywhere on heading-source validity, holdover
state, or the heading-selection code path at all. This was already true
before this phase (the two computations were always independent
`if` blocks) - the test makes it explicit and regression-proof.

**Stale heading isn't held indefinitely.**
Confirmed by test (`test_holdover_expires_after_max_duration_and_becomes_lost`,
`test_stale_heading_not_held_indefinitely`,
`test/test_heading_holdover/test_main.cpp`): holdover tracks elapsed time
since the loss began and transitions to `Lost` (reports invalid) once
`maxHoldoverMs` is exceeded, regardless of how long the loss continues
afterward.

**Holdover duration is configurable.**
New config field `icmHdgHoldMs` (`icm20948Config.json`), read every cycle
into `ImuCycleInput::headingHoldoverConfig.maxHoldoverMs`. Range 0-60000ms,
default 5000ms (provisional - no hardware available this session to tune
against real DMP/compass dropout characteristics). 0 disables holdover
entirely (every loss immediately reports invalid, matching pre-Phase-8
behavior byte-for-byte).

**Quality downgrades during holdover.**
Confirmed by test (`test_quality_downgrades_to_poor_during_holdover`):
`ImuHeadingHoldover::Result::quality` is `HeadingQuality::Poor` for the
entire time the module isn't in a fully-confirmed `Tracking` state
(covers both active dead-reckoning and the recovery blend before a
newly-valid source is fully trusted again).

**Recovery requires consecutive valid samples.**
Confirmed by test (`test_recovery_requires_consecutive_valid_samples`):
after a loss, `minConsecutiveSamplesToRecover` (default 3, matching
`DmpValidator`'s own convention) consecutive valid samples are required
before the module leaves `Holdover`/`Lost` and resumes `Tracking` - a
single flickering good sample doesn't cause an immediate snap back. Not
exposed as a separate config field (kept as a fixed default) - only the
overall holdover duration was asked to be configurable.

Deliberate exception: the very FIRST fix ever (before anything has been
tracked) is NOT gated by this rule - it snaps immediately, matching the
precedent `HeadingSourceSelector` already uses for "first valid source
after having none." "Recovery" implies recovering from a loss; requiring
several consecutive samples before trusting the very first-ever reading
would just be redundant with `DmpValidator`'s own
`minConsecutiveValidSamples` gate and would delay startup for no benefit.
See `ImuHeadingHoldover.cpp`'s `everTracked` handling and
`test_tracks_directly_when_source_valid`.

**Transitions use circular interpolation.**
Confirmed by test (`test_recovery_uses_circular_interpolation_not_instant_jump`):
recovering from `Holdover` (not `Lost` - see below) blends from the held
heading to the recovered source's heading via
`ImuAngleMath::circularInterpolate` over `recoveryBlendMs` (default
1000ms, fixed), the same function `HeadingSourceSelector` already uses
for its own inter-source transitions.

Recovering from `Lost` snaps directly instead of blending - there is no
continuous position worth preserving after the holdover window has
already fully expired, the same reasoning `HeadingSourceSelector` uses
for "first valid source after having none." See
`test_lost_recovery_snaps_without_blend`.

**North crossing doesn't jump 360°.**
Confirmed by test (`test_north_crossing_during_recovery_does_not_jump_360`):
a recovery blend crossing the 350°→10° arc takes the short way through
0/360 rather than the long way through 180°, inherited correctly from
`ImuAngleMath::circularInterpolate`'s existing shortest-path behavior
(already relied on elsewhere in this codebase - not new math, just a new
caller of it).

**Magnetic disturbance doesn't immediately poison gyro-only continuation.**
Confirmed by design and by test
(`test_magnetic_disturbance_loss_still_allows_holdover`):
`ImuHeadingHoldover::update()` takes only a `bool sourceValid` - it has no
visibility into *why* the selector reported invalid (magnetic
disturbance, DMP staleness, a bad quaternion, anything else), so there is
no code path by which a disturbance-caused loss could be treated
differently from any other loss. This is enforced structurally, not just
by convention: the module's input type simply doesn't carry a reason
code.

**Gyro-only continuation is time-limited and clearly identified.**
Time-limited: see "stale heading isn't held indefinitely" above.
Clearly identified: `ImuCycleOutput::headingHoldover` (bool) and
`headingHoldoverState` (`HeadingHoldoverState::Tracking`/`Holdover`/`Lost`)
are new fields threaded all the way to the web UI (`icm20948.js`'s
"Holdover (gyro-only continuation)" row, showing state and explicitly
noting it is NOT sent as PGN 127250 when active) and to
`Icm20948WebData`'s JSON (`headingHoldover`/`headingHoldoverState` on the
`data` endpoint).

## Known gap, deliberately deferred

**Holdover state is not yet added as a CSV diagnostic-capture column.**
Adding one would require touching three already-built, already-tested,
interdependent subsystems in the same change: `ImuDiagnostics.h`/`.cpp`'s
41-column format (shared with `test_diagnostics`), the Python calibration
tool's `csv_loader.py` (`EXPECTED_COLUMNS`), and the replay tool's
own column-index-based parser (`test/test_replay_tool/test_main.cpp`) -
plus regenerating the committed fixture
(`tools/icm20948_replay/fixtures/level360Rotation.csv`). Given the
remaining session scope (release-candidate build and final report still
ahead), this was judged not worth the risk of a schema change touching
that much surface area this late, especially un-verifiable on real
hardware this session either way. The web UI (the primary, fully-tested
surface for this requirement) already satisfies "clearly identified."
During a holdover CSV capture today, `active_heading_source` reads
`none` while `output_heading_deg` is still a valid (non -1) number - an
implicit, if not ideal, signal for anyone parsing a capture by hand.

## An important characterization, not a bug

Holdover only ever actually *engages* when the selector's `desired`
source can become `None` - and reading `ImuHeadingSource.cpp` and
`ImuCycleProcessor.cpp` closely, that is only possible in ONE of the four
heading-mode settings:

- **`dmp`** (the default): `desired = dmp.valid ? Dmp : None`. DMP has a
  real, ongoing invalidation path via `DmpValidator` (staleness, bad
  quaternion, disagreement with compass, sudden jump) that can trip at
  any time after initial acquisition. **This is the mode holdover was
  built for, and it happens to be the default one.**
- **`software_compass`**: `desired = compass.valid ? SoftwareCompass : None`
  - but `compassCandidate` is constructed as
  `SourceCandidate(true, rawCompassHeadingDeg, ...)` in
  `ImuCycleProcessor.cpp` - **always `valid=true`, unconditionally**. A
  magnetic disturbance downgrades its `quality` to `Poor` but never its
  validity. `desired` can never be `None` in this mode - holdover cannot
  engage.
- **`software_9axis_fusion`**: `desired = fusion.valid ? SoftwareFusion : None`,
  and `fusionValid` is computed as just
  `(nowMs - taskStartMs) > 3000` - a one-time startup settle timer with
  no ongoing invalidation path at all. Once past 3 seconds after task
  start, fusion never becomes invalid again. Holdover can only
  theoretically engage during that initial 3-second window (not really a
  "loss after tracking" scenario) - functionally unreachable in this mode.
- **`auto`/`diagnostic_only`**: `pickAuto()` falls back through
  fusion → DMP → compass, and since compass is unconditionally valid,
  `pickAuto()` never returns `None` either.

So in practice, **holdover is only reachable in explicit `dmp` mode** -
which is this task's default (`icmHeadingMode` defaults to `"dmp"`, per
`doc/IcmImplementationAudit.md`), so it's live out of the box, not an
opt-in most installations would need to discover. Every integration test
above exercises `HeadingSourceMode::Dmp` specifically for this reason.
This asymmetry (only DMP has an ongoing "can become invalid after being
valid" characteristic; compass never invalidates and fusion only during
startup) is a genuine property of the existing validation design, not
something this phase changed - worth understanding clearly before
physical testing, since it means holdover activity should be expected
around DMP staleness/disagreement events specifically, not around
magnetic disturbance in `auto`/compass-only configurations (those still
show a quality downgrade, just not a holdover engagement).
