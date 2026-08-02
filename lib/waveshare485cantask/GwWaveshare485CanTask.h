#pragma once
/*
  Hardware definitions for the Waveshare ESP32-S3-RS485-CAN board
  https://www.waveshare.com/wiki/ESP32-S3-RS485-CAN

  This board is used as a WiFi <-> NMEA2000 (and optionally NMEA0183/RS485)
  gateway on the boat. It is externally powered (7-36V screw terminal or
  USB-C) - it does NOT draw power from the NMEA2000 bus.

  Pin assignments below are cross-checked against three independent sources:
   - Waveshare's own demo firmware (WS_GPIO.h/WS_CAN.h/WS_RS485.h from
     https://files.waveshare.com/wiki/ESP32-S3-RS485-CAN/ESP32-S3-RS485-CAN-Demo.zip)
   - The board schematic (ESP32-S3-RS485-CAN-Schematic.pdf, same wiki page)
   - The wiki's own pinout table
  See ../../doc/WaveshareRs485Can.md in this repo for the full writeup.

  Following the "Own Hardware" pattern documented in doc/Hardware.md /
  lib/exampletask/Readme.md: this file lives outside the core so that
  `git fetch upstream && git merge upstream/master` applies cleanly.
*/
/*
  Three board defines share this one header, since they're the same
  physical board with different things wired to the Qwiic expansion
  connector:
   - BOARD_WAVESHARE_ESP32S3_RS485_CAN: ICM-20948 IMU on I2C (lib/icm20948task)
   - BOARD_WAVESHARE_ESP32S3_RS485_CAN_GARMIN: external TTL<->RS422
     transceiver on a plain UART, bridging a Garmin chartplotter's NMEA0183
     port onto this device's NMEA2000 bus (no custom task needed - see the
     Serial2 block below, it's the core's existing second-channel mechanism),
     plus a Calypso Ultrasonic wind sensor over BLE (lib/calypsotask)
   - BOARD_WAVESHARE_ESP32S3_RS485_CAN_AIS: direct TTL UART connection to
     an AIS unit's own UART (receive-only, no custom task needed either -
     same Serial2 mechanism, AIS's "!--VDM"/"!--VDO" sentences are already
     recognized by the core's generic NMEA0183 line parser like any other
     sentence)
  These never coexist in one build, so all three can define the same two
  GPIOs for different purposes without conflict.
*/
#if defined(BOARD_WAVESHARE_ESP32S3_RS485_CAN) || defined(BOARD_WAVESHARE_ESP32S3_RS485_CAN_GARMIN) || defined(BOARD_WAVESHARE_ESP32S3_RS485_CAN_AIS)

// Native USB Type-C -> S3's built-in USB CDC. With ARDUINO_USB_CDC_ON_BOOT,
// the Arduino core routes the ordinary Serial object over USB CDC, but
// core code (src/main.cpp) logs via a symbol named USBSerial - GwHardware.h
// only aliases that for boards it knows about (M5Atom/StickC/NodeMCU/C3),
// not our generic esp32-s3-devkitm-1 board id, so we alias it ourselves the
// same way the C3 case does.
#ifdef ARDUINO_USB_CDC_ON_BOOT
  #if ARDUINO_USB_CDC_ON_BOOT == 1
    #define USBSerial Serial
  #endif
#endif

// --- CAN / NMEA2000 (TWAI controller -> TJA1051T/3/1J transceiver) ---
// Confirmed: WS_GPIO.h TXD2=15/RXD2=16, schematic net labels TXD2/RXD2 on
// the CAN block feeding U8 (TJA1051T/3/1J). No direction pin needed - the
// transceiver has no DE/RE, it's inherently bus-driving.
#define ESP32_CAN_TX_PIN GPIO_NUM_15
#define ESP32_CAN_RX_PIN GPIO_NUM_16

// --- RS485 (UART1 -> SP3485EN transceiver), optional NMEA0183 ---
// Confirmed: WS_GPIO.h TXD1=17/RXD1=18/TXD1EN=21, schematic shows GPIO21
// tied to both RE and DE on U7 (SP3485EN) - i.e. one pin, active HIGH
// enables the driver (transmit), LOW enables the receiver.
//
// IMPORTANT: the core's static enable-pin logic (see
// createSerialImpl() in lib/channel/GwChannelList.cpp) only knows how to
// drive GWSERIAL_ENA for GWSERIAL_TYPE_UNI / _RX / _TX - there is no case
// for GWSERIAL_TYPE_BI. If you set this to BI, GWSERIAL_ENA is silently
// left unconfigured and the RS485 driver's enable state is undefined -
// don't do that with this wiring. UNI mode lets you pick "send" or
// "receive" per-session from the web UI, which is the same limitation the
// upstream M5 Tail485 setup already documents (doc/Hardware.md).
#define GWSERIAL_RX GPIO_NUM_18
#define GWSERIAL_TX GPIO_NUM_17
#define GWSERIAL_ENA GPIO_NUM_21
#define GWSERIAL_ELO 0
#define GWSERIAL_TYPE GWSERIAL_TYPE_UNI

// Externally powered (screw terminal / USB-C), no current drawn from the
// NMEA2000 bus itself - LEN stays 0 (the core's own default).
#define N2K_LOAD_LEVEL 0

// BOOT button (GPIO0, schematic "KEY" block) doubles as a runtime button
// post-boot, same pattern already used by BOARD_HOMBERGER.
#define GWBUTTON_PIN GPIO_NUM_0
#define GWBUTTON_ACTIVE LOW
#define GWBUTTON_PULLUPDOWN

// --- Qwiic-compatible expansion connector ---
// Confirmed via the schematic's own GPIO summary table: IO1/IO2 have no
// other onboard function (not CAN/RS485/RTC/SD/network/relay/DIN/"Other") -
// they're genuinely spare GPIOs broken out to a JST-SH Qwiic connector.
// What's wired to them differs per physical unit/board define below.

#ifdef BOARD_WAVESHARE_ESP32S3_RS485_CAN
// ICM-20948 IMU (lib/icm20948task) via I2C. SDA/SCL assignment is a guess
// matching the standard Qwiic pin order (GND,3V3,SDA,SCL); since the S3's
// I2C peripheral is routed through its GPIO matrix (not fixed-function
// pins), if this is backwards the IMU simply won't answer on the bus -
// swap the two values and reflash, no hardware risk either way.
#define GWICM20948_SDA_PIN GPIO_NUM_2
#define GWICM20948_SCL_PIN GPIO_NUM_1

// DST810 depth/speed/temp sensor over BLE (lib/dst810task) - no pins
// needed, just enables the BLE-central task on this unit.
#define GWDST810_ENABLE
#endif

#ifdef BOARD_WAVESHARE_ESP32S3_RS485_CAN_GARMIN
// External TTL<->RS422 transceiver breakout, bridging a Garmin
// chartplotter's NMEA0183 (RS422) port onto this device's NMEA2000 bus.
// This is the core's existing second serial channel (SERIAL2_CHANNEL_ID,
// see lib/channel/GwChannelList.cpp) - no custom task/lib needed, it gets
// the same automatic NMEA0183<->NMEA2000 conversion and web config/status
// fields as the onboard RS485 port (Serial1) does.
//
// Wiring (confirmed on the physical harness):
//   orange wire (IO2/GPIO2) -> RS422 module RXD (this is our UART TX out)
//   yellow wire (IO1/GPIO1) -> RS422 module TXD (this is our UART RX in)
// Type BI (not UNI like the onboard half-duplex RS485 chip): the external
// RS422 transceiver has independent driver/receiver circuits that are
// always both active - no shared pair, no direction/enable pin to manage.
// Garmin configured for its "high speed" NMEA0183 setting -> 38400 baud.
//
// Baud deliberately NOT fixed via GWSERIAL2_BAUD here (unlike the AIS
// board's Serial2, which is): leaving it undefined means
// lib/channel/GwChannelList.cpp's preinit() skips locking the config
// item read-only, so "serial2 baud rate" is a normal editable field on
// the web UI's Config page (category "serial2 port") - useful while
// bench-testing whether a slower rate is more tolerant of a marginal
// RS422 link. Whatever's picked there must still match the Garmin's own
// NMEA0183 port speed setting on the plotter itself.
#define GWSERIAL2_TX GPIO_NUM_2
#define GWSERIAL2_RX GPIO_NUM_1
#define GWSERIAL2_TYPE GWSERIAL_TYPE_BI

// This unit's whole purpose is being a GPS/position source on N2K (the
// Garmin's position data, bridged in over the RS422 link above) - declare
// it as such (NMEA2000 Class 60 "Navigation", Function 145 "Own Vessel
// Position (GNSS)") rather than the default generic-gateway identity, so
// other devices' "search for a GPS source" UI (e.g. a VHF radio's NMEA2000
// setup) actually finds it - that search filters by declared class/
// function, not just by which PGNs are present on the bus.
#define N2K_DEVICE_CLASS 60
#define N2K_DEVICE_FUNCTION 145

// Calypso Ultrasonic wind sensor over BLE (lib/calypsotask) - no pins
// needed, just enables the BLE-central task on this unit. BLE-central
// (Calypso) and WiFi already coexist fine on this chip - proven by the
// IMU unit's DST810 integration, same ESP32-S3.
#define GWCALYPSO_ENABLE
#endif

#ifdef BOARD_WAVESHARE_ESP32S3_RS485_CAN_AIS
// Direct TTL UART connection to an AIS unit's own UART output (no RS422
// transceiver in between this time - the AIS unit's UART pins are
// already at logic level). Receive-only for now: we just want to verify
// the link works and AIS sentences are actually arriving, not send
// anything back to the unit.
//
// Wiring: TENTATIVE, not yet bench-confirmed - swap the two values and
// reflash if no data shows up (same zero-risk swap-and-retry the Garmin
// bridge's RS422 link needed before it was confirmed correct).
//   yellow wire (IO1/GPIO1) -> AIS unit's TXD (this is our UART RX in)
//   orange wire (IO2/GPIO2) -> AIS unit's RXD (unused for now, receive-only)
// AIS transponders transmit NMEA0183 at the IEC 61162-1/61993-2
// "high speed" rate, 38400 baud - not the usual 4800.
//
// Bring-up finding (2026-07): neither wiring orientation, with or without
// GWSERIAL2_INVERT (see GwChannelList.cpp) forced on, ever produced a
// single valid or garbage line - one orientation was pure silence, the
// other constant UART BREAK errors with zero decoded bytes either way.
// That pattern (activity that never resolves into any framed byte,
// regardless of polarity) doesn't fit a wiring/polarity problem - it's
// more consistent with the AIS unit's UART output not being alive at
// all. Suspected fried AIS unit; it and this gateway have both been
// disconnected pending hardware repair/replacement, so these pin/baud
// values are unverified until then.
#define GWSERIAL2_RX GPIO_NUM_1
#define GWSERIAL2_TX GPIO_NUM_2
#define GWSERIAL2_TYPE GWSERIAL_TYPE_RX
#define GWSERIAL2_BAUD 38400
#endif

#endif
