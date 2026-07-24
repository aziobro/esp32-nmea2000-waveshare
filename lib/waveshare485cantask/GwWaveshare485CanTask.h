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
#ifdef BOARD_WAVESHARE_ESP32S3_RS485_CAN

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

#endif
