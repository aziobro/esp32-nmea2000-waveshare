#pragma once
#include "GwApi.h"

/*
  Passive RS485 Modbus RTU sniffer for an EG4 battery already being polled
  by an inverter on the same bus - this task never transmits, it only
  listens (see GwWaveshare485CanTask.h's GWEG4BATTERY_ENA_PIN comment: held
  permanently in receive mode). Frame sync is inter-byte-silence + CRC16
  (lib/modbussniffpure/ModbusRtuSniffer), since Modbus RTU has no delimiter
  byte of its own.

  Phase 1 only: captures raw frames (timestamp, CRC valid/invalid, hex
  bytes) to a downloadable buffer for reverse-engineering the register map
  from real traffic - no register decoding or NMEA2000 publishing yet,
  since no EG4/Growatt protocol documentation is in hand. See doc/ or the
  project history for the follow-up phase once a real capture is available.

  Enabled via GWEG4BATTERY_ENABLE, defined by the board header for
  whichever physical unit has this feature (see GwWaveshare485CanTask.h) -
  inactive (does nothing) if a board doesn't define it.
*/
void initEg4Battery(GwApi *api);
DECLARE_INITFUNCTION(initEg4Battery);
