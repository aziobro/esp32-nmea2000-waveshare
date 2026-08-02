# Heading mode semantics (Phase 9 verification)

Phase 9 of the post-Request-B work: "reconsider default heading mode" -
verify `icmHeadingMode`'s meanings match what the user specified, confirm
DMP is genuinely validated (not unvalidated legacy) in `dmp` mode, and
decide whether a separate unvalidated `legacy_dmp` mode is needed. This
is primarily a verification pass against the pipeline as it now stands
after Phases 1-8, not new functionality - one real gap was found and is
documented rather than fixed (see "Compass and fusion validation depth"
below), consistent with this branch's practice of disclosing gaps rather
than quietly living with them or overclaiming a fix that hasn't been
hardware-verified.

## Required meaning vs actual behavior

| Mode | Required meaning | Verified behavior |
|---|---|---|
| `dmp` | validated-DMP-only | **Confirmed.** `desired = dmp.valid ? Dmp : None` (`ImuHeadingSource.cpp`), and `dmp.valid` comes from `DmpValidator::validate()` - staleness (`HR_DMP_STALE`), quaternion sanity (`HR_BAD_QUATERNION`), disagreement with the independently-computed compass (`HR_DMP_COMPASS_DISAGREE`), sudden implausible jump (`HR_SUDDEN_JUMP`), a minimum-consecutive-good-samples gate, and (Phase 8) magnetic-disturbance rejection via `HR_MAG_FIELD_CHANGE`. This is this task's *default* mode (`icmHeadingMode` defaults to `"dmp"`) - the original bug this whole rewrite exists to fix (DMP yaw sent unconditionally, bypassing calibration) cannot recur through this mode. |
| `software_compass` | validated-compass-only | **Partially - see finding below.** No automatic fallback to another source (`desired = compass.valid ? SoftwareCompass : None`), but `compass.valid` is unconditionally `true` - the compass candidate never actually reports invalid, only its `quality` downgrades to `Poor` during a magnetic disturbance. |
| `software_9axis_fusion` | validated-fusion-only | **Partially - see finding below.** No automatic fallback, but `fusionValid` is only a one-time startup settle timer (`(nowMs - taskStartMs) > 3000`) with no ongoing validation of the fusion filter's own output. |
| `auto` | policy-based (prefer fusion, then DMP, then compass) | **Confirmed**, unchanged from Phase 1. `pickAuto()` in `ImuHeadingSource.cpp`. |
| `diagnostic_only` | compute but don't transmit | **Confirmed**, unchanged from Phase 1. Computed identically to `auto`; `GwIcm20948Task.cpp`'s send gate explicitly excludes `HeadingSourceMode::DiagnosticOnly`. |

## No `legacy_dmp` mode added

Not needed: `dmp` mode is fully validated (see above), so there is no
"unvalidated legacy" behavior left to give a separate name to. Adding one
anyway would just be new surface area with no purpose - per the user's
own "prefer not to add unless necessary."

## `icmSendHdg` still defaults off

Confirmed unchanged (`"default": "false"` in `icm20948Config.json`) -
this phase didn't touch it, matching the explicit instruction not to
enable heading transmission by default.

## Compass and fusion validation depth (finding, not fixed)

DMP's validation is meaningfully deeper than compass's or fusion's:

- **Compass** never reports invalid, no matter how implausible the raw
  magnetometer reading is (a dropout reading all-zero, a wildly
  out-of-range magnitude) - `ImuMagMonitor`'s existing
  `minMagnitude`/`maxMagnitude` check already *detects* this (feeds into
  `MagDisturbanceState::Disturbed`), but that only downgrades
  `quality` to `Poor`, it never sets `compassCandidate.valid = false`.
- **Fusion** has no ongoing validation at all past its 3-second startup
  window - no NaN/divergence check on the filter's own quaternion output.

This means, per `doc/IcmHeadingValidityAudit.md`'s finding, Phase 8's new
holdover mechanism is only reachable in `dmp` mode: `software_compass`
and `software_9axis_fusion` can never make `HeadingSourceSelector` report
`None` after acquiring a fix, so holdover never has anything to bridge in
those modes.

**Deliberately not fixed this session.** Retrofitting genuine
invalidation into compass/fusion (e.g. rejecting a candidate outright
below some magnitude threshold, or adding a fusion output sanity check)
is exactly the kind of behavior change that needs verification against a
real sensor's actual noise characteristics before shipping - an
overly-aggressive threshold could make a mode that currently works
(if imprecisely) start reporting "no heading" on a real, if noisy, unit.
Per this branch's explicit constraint ("do not claim hardware accuracy
based on synthetic tests"), this is flagged for a follow-up *after*
physical bring-up, once real compass/fusion noise/failure characteristics
are known, rather than guessed at now.
