#pragma once
#include "GwApi.h"

/*
  Task for an ICM-20948 IMU on the Waveshare ESP32-S3-RS485-CAN board's
  onboard Qwiic-compatible connector.

  Sends heel/pitch as NMEA2000 PGN 127257 (Attitude), computed from the
  accelerometer only (simple tilt calculation) - no magnetometer/heading yet,
  see the .cpp for why.

  Pins come from GWICM20948_SDA_PIN/GWICM20948_SCL_PIN, defined by the board
  (see lib/waveshare485cantask/GwWaveshare485CanTask.h). This task is inactive
  (does nothing) if a board does not define them.
*/
void initIcm20948(GwApi *api);
DECLARE_INITFUNCTION(initIcm20948);
