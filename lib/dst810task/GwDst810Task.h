#pragma once
#include "GwApi.h"

/*
  BLE-central bridge for an Airmar/B&G DST810 depth/speed/water-temp
  sensor. The DST810 advertises as a BLE peripheral (name prefix "DST")
  exposing a Nordic UART Service; no pairing needed to read. It streams
  plain NMEA0183 text ($SDDPT depth, $YXMTW water temp, $VMVHW speed
  through water, $VMVLW log distance) which we parse and convert to
  NMEA2000 PGNs the same way any other task here calls
  api->sendN2kMessage().

  Enabled via GWDST810_ENABLE, defined by the board header for whichever
  physical unit has this feature (see GwWaveshare485CanTask.h) - inactive
  (does nothing) if a board doesn't define it.
*/
void initDst810(GwApi *api);
DECLARE_INITFUNCTION(initDst810);
