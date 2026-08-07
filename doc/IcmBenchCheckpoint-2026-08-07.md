# ICM-20948 bench checkpoint - 2026-08-07

This is the end-of-night checkpoint for the IMU reliability and compass-heading
work on the Waveshare ESP32-S3 device at `http://192.168.8.156`.

## Code state

- Removed the onboard DMP heading path from the active IMU implementation.
  Heading sources are now software-only: diagnostic-only, software compass,
  software 9-axis fusion, and auto.
- Removed the SparkFun and Wollewald library build dependencies from the
  Waveshare firmware path. The hardware adapter now uses the direct-register
  ICM-20948 backend.
- Kept the magnetometer on the direct AGMT/read path with bounded retries,
  I2C recovery, communication counters, and live web diagnostics.
- Fixed the software fusion yaw sign convention so the fusion heading matches
  the compass heading convention.
- Fixed gyro calibration save behavior so saving no longer clears the just
  collected result from the UI.
- Left the I2C bus clock at `50 kHz` in
  `lib/icm20948task/GwIcm20948HardwareAdapter.cpp`. This is currently the
  reliability baseline.

## Hardware test results

Three I2C speeds were tried with the direct-register backend:

- `400 kHz`: firmware produced valid samples, but `sensorErrorCount`,
  `i2cRecoveryCount`, and `i2cRetryCount` climbed continuously. Not suitable.
- `100 kHz`: better than `400 kHz`, but startup was rough and retry count kept
  climbing. Marginal.
- `50 kHz`: best result. A small number of startup recoveries still appeared,
  but recovery count flattened after startup and the IMU continued reporting
  valid `software_9axis_fusion` data.

Final live check after flashing the `50 kHz` build:

- `valid=true`
- `headingSource=software_9axis_fusion`
- `headingQuality=good`
- `sensorErrorCount` / `i2cRecoveryCount` settled at `12`
- Final 5-sample window held recovery count flat at `12 -> 12`
- `i2cRetryCount` drifted only slightly from `78 -> 80`

The normal PlatformIO upload failed twice during the `460800` baud handoff.
The board was recovered and flashed successfully with direct `esptool.py` at
`115200` using `--no-stub`.

## Verification

- `pio test -e icm20948_native_test`: passed, `172/172`
- `pio run -e waveshare-esp32s3-rs485-can`: passed
- `pio run -e waveshare-esp32s3-rs485-can-garmin`: passed earlier in this
  checkpoint series
- `pio run -e waveshare-esp32s3-rs485-can-ais`: passed earlier in this
  checkpoint series
- Device flash: successful with the `50 kHz` build

## Current interpretation

The direct-register software-fusion path is working, and the IMU can provide
valid fused heading data. The remaining communication symptoms look timing or
bus-health related, not a DMP/library issue. The device is usable at `50 kHz`,
but faster bus clocks are not reliable on this wiring/device combination.

Before increasing internal sample/fusion rate, keep the bus at `50 kHz` and
watch retry/recovery counters during a longer stationary run. If the counters
stay flat after startup, the next code experiment should decouple internal
sensor/fusion sampling from NMEA send rate: sample/fuse at roughly `25 Hz`,
send NMEA at `10 Hz`, and keep the web UI reading the latest processed sample.

## Open follow-ups

- Run a longer stationary soak at `50 kHz` and confirm recovery count stays flat
  after startup.
- Improve or tune gyro bias calibration; stationary ROT still wanders enough to
  affect fusion confidence.
- Add separate config values for internal sample/fusion rate and NMEA output
  rate.
- Consider a hardware-side review of I2C pull-ups, cable length, and reset/power
  sequencing if retries remain visible at `50 kHz`.
- Keep raw `calibration data/` captures local unless deliberately archiving a
  calibration dataset in git.
