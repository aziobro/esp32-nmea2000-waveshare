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
  Two board defines share this one header, since they're the same physical
  board with different things wired to the Qwiic expansion connector:
   - BOARD_WAVESHARE_ESP32S3_RS485_CAN: ICM-20948 IMU on I2C (lib/icm20948task)
   - BOARD_WAVESHARE_ESP32S3_RS485_CAN_GARMIN: external TTL<->RS422
     transceiver on a plain UART, bridging a Garmin chartplotter's NMEA0183
     port onto this device's NMEA2000 bus (no custom task needed - see the
     Serial2 block below, it's the core's existing second-channel mechanism)
  The two never coexist in one build, so both can define the same two GPIOs
  for different purposes without conflict.
*/
#if defined(BOARD_WAVESHARE_ESP32S3_RS485_CAN) || defined(BOARD_WAVESHARE_ESP32S3_RS485_CAN_GARMIN)

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
#define GWSERIAL2_TX GPIO_NUM_2
#define GWSERIAL2_RX GPIO_NUM_1
#define GWSERIAL2_TYPE GWSERIAL_TYPE_BI
#define GWSERIAL2_BAUD 38400
#endif

#endif
