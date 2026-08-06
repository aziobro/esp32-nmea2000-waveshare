# ICM-20948 compass bench status - 2026-08-06

This captures the current stopping point from live bench testing against the
device at `http://192.168.8.156`.

## Firmware changes in this checkpoint

- Switched the Waveshare ESP32-S3 IMU path to the Wollewald-style direct raw
  ICM-20948 reader for the driver bake-off.
- Added bounded I2C retries/recovery, magnetometer validity counters, and live
  diagnostics for raw/boat/corrected magnetometer vectors.
- Fixed the generic config POST parser so overlong values are skipped instead
  of saved as truncated suffixes. This was corrupting `icmCalJson` when normal
  config saves included the advanced calibration blob.
- Changed `GwConfigHandler::updateValue()` to refresh the in-memory config
  after persisting to NVS. Calibration imports and offset/deviation updates now
  apply live without a restart.
- Removed forced ESP32 restarts from IMU calibration-control saves. This avoids
  the repeatable failure where a firmware/config restart left the external IMU
  not ACKing until the IMU/Qwiic connection was power-cycled.

## Verified on hardware

- Native tests pass: `pio test -e icm20948_native_test` reports 187/187 passed.
- Firmware builds and uploads for `waveshare-esp32s3-rs485-can`.
- The IMU can recover by unplug/replug of the IMU/Qwiic side when an ESP32 reset
  leaves it not ACKing on I2C.
- Calibration updates now apply live through
  `/api/user/icm20948Task/calControl` without forcing a device restart.
- The advanced calibration stays active after import/offset/deviation changes:
  `magCorr` differs from `magBoat`, and `magMagnitude` is typically about
  46-50 uT in the current test location.

## Current live compass configuration

- Mount orientation: `1` (`Starboard`)
- Heading source mode: `software_compass`
- Heading invert: `true`
- Fixed heading offset: `162.74`
- Magnetometer calibration quality: `0.898687`
- Deviation table: enabled

Current deviation table:

```json
[
  {"measuredHeadingDeg": 7.7, "correctionDeg": -7.7},
  {"measuredHeadingDeg": 52.5, "correctionDeg": 17.5},
  {"measuredHeadingDeg": 115.0, "correctionDeg": 65.0},
  {"measuredHeadingDeg": 249.0, "correctionDeg": 21.0},
  {"measuredHeadingDeg": 356.0, "correctionDeg": 4.0}
]
```

Note: the east correction was last recommended to be reduced from `17.5` to
about `8.0` degrees after a live east check that still read about 99-101
degrees. That final adjustment had not yet been applied when this note was
written.

## Current compass observations

With calibration active and heading invert enabled:

- North was stabilized at the 359/0 wrap boundary after adding both north-side
  anchors (`7.7 -> -7.7`, `356 -> +4`).
- East was still about 8-10 degrees high before the final recommended east
  table adjustment.
- South previously measured about 115 degrees before deviation correction.
- West reached about 267 degrees with the four-point deviation table when held
  level and steady.

The remaining nonlinearity is too large for a fixed offset alone. The current
table is suitable as a bench workaround, but a proper compass swing with more
points should replace it before navigation use.

## Open issues

- ESP32-side resets can leave the external ICM-20948 not ACKing on SDA=2/SCL=1
  until the IMU/Qwiic side is unplugged/replugged. Avoiding calibration-triggered
  restarts reduces the impact, but startup robustness still needs a hardware or
  bus-reset follow-up.
- I2C recovery counters climb during live testing. Communication is usable, but
  the count is a useful warning signal for cable, pull-up, timing, or clock
  stretching investigation.
- The current deviation table was created from hand-held cardinal checks. It
  should be replaced by a steady 8- or 12-point compass swing against the
  external compass.
- Do not commit the raw `calibration data/` captures unless explicitly needed;
  keep them as local bench artifacts.
