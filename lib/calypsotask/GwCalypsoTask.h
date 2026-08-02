#pragma once
#include "GwApi.h"

/*
  BLE-central bridge for a Calypso Ultrasonic (LCJ Capteurs) wireless
  masthead wind sensor. Protocol per the maritime-labs/calypso-anemometer
  reference project (see GwCalypsoTask.cpp for the full writeup) - every
  unit advertises the identical literal name "ULTRASONIC" (no per-unit
  suffix, unlike the DST810), and streams a 10-byte raw-binary reading
  (not NMEA0183 text) over a notify characteristic aliased to the
  standard-form UUID 0x2A39. No pairing, no GATT writes needed to start
  receiving data.

  Enabled via GWCALYPSO_ENABLE, defined by the board header for whichever
  physical unit has this feature (see GwWaveshare485CanTask.h) - inactive
  (does nothing) if a board doesn't define it.
*/
void initCalypso(GwApi *api);
DECLARE_INITFUNCTION(initCalypso);
