Waveshare ESP32-S3-RS485-CAN
============================
Hardware: [Waveshare ESP32-S3-RS485-CAN](https://www.waveshare.com/wiki/ESP32-S3-RS485-CAN)

Build Define: `BOARD_WAVESHARE_ESP32S3_RS485_CAN`

PlatformIO Environment: `waveshare-esp32s3-rs485-can`

Definitions live in [../lib/waveshare485cantask/](../lib/waveshare485cantask/) rather
than in the shared `GwHardware.h`, following the "Own Hardware" pattern in
[Hardware.md](Hardware.md) - this keeps `git fetch upstream && git merge
upstream/master` conflict-free.

Why this board
---------------
Used as the boat's NMEA2000-WiFi gateway. It's externally powered (7-36V DC
screw terminal or USB-C) rather than parasitic on the N2K bus, has isolated
CAN and RS485 transceivers, and a 120R termination jumper on each bus - no
extra glue hardware needed on either interface.

Hardware summary
-----------------
* MCU: ESP32-S3R8 (bare chip, not a WROOM module) - dual-core, WiFi/BLE, **8MB
  octal PSRAM on-chip**
* Flash: 16MB external SPI (W25Q128JVSI)
* USB: Type-C wired directly to the S3's native USB pins (no USB-UART bridge
  chip) - needs `ARDUINO_USB_CDC_ON_BOOT`/`ARDUINO_USB_MODE` for the serial
  console
* CAN transceiver: TJA1051T/3/1J (no direction pin required)
* RS485 transceiver: SP3485EN, single combined RE/DE direction pin
* RTC: PCF85063AT with battery backup connector (not currently used by this
  build define)
* USB, CAN and power indicator LEDs are plain bi-color LEDs, not
  addressable/FastLED - `GWLED_*` is intentionally left undefined for this
  board

Pin mapping
-----------
| Signal              | GPIO | Notes |
|---------------------|------|-------|
| CAN TX (TWAI)        | 15   | to TJA1051T |
| CAN RX (TWAI)        | 16   | from TJA1051T |
| RS485 TX (UART1)     | 17   | to SP3485EN DI |
| RS485 RX (UART1)     | 18   | from SP3485EN RO |
| RS485 direction      | 21   | drives SP3485EN RE+DE together |
| BOOT/user button     | 0    | schematic "KEY" block, same pattern as `BOARD_HOMBERGER` |

Sources (cross-checked, not guessed)
-------------------------------------
1. Waveshare's own demo firmware headers (`WS_GPIO.h`, `WS_CAN.h`,
   `WS_RS485.h`) from the demo zip linked on the
   [wiki page](https://www.waveshare.com/wiki/ESP32-S3-RS485-CAN) -
   authoritative because it's Waveshare's own working code for this exact
   board (`MAIN_WIFI_STA.ino` prints the SSID "ESP32-S3-RS485-CAN").
2. The board schematic (`ESP32-S3-RS485-CAN-Schematic.pdf`, same wiki page)
   - confirms transceiver part numbers, that RE and DE on the SP3485EN share
   one GPIO, and that CAN passes through the same digital isolator (U9) as
   RS485.
3. The wiki's prose/pinout table (used only to corroborate - the flattened
   HTML table on that page mis-orders columns, so it was not trusted alone).

RS485 direction polarity - verify before relying on it
---------------------------------------------------------
Schematic net "RS485_EN" ties SP3485EN pins RE (2) and DE (3) together, so
one GPIO controls direction: HIGH should enable the driver (transmit) and
disable the receiver (RE is active-low), LOW should do the opposite. That's
the standard wiring convention and matches how the ESP-IDF UART's built-in
RS485 half-duplex mode drives this kind of pin (which is what Waveshare's
own demo firmware uses). `GWSERIAL_ELO` is set to `0` (active-high enable)
on that assumption.

This project's core does **not** use the ESP-IDF RS485 half-duplex UART
mode - `createSerialImpl()` in `lib/channel/GwChannelList.cpp` only sets the
enable pin to a fixed level once, based on channel type, and it has no
handling at all for `GWSERIAL_TYPE_BI` (bidirectional). That's why this
board is configured as `GWSERIAL_TYPE_UNI`: you pick "send" or "receive" for
the RS485 port once, from the web UI, for the session - same limitation the
M5 Tail485 setup already has (see Hardware.md). Don't switch this to `_BI`
with this wiring; the direction pin would be left floating.

Before trusting the RS485 port with real NMEA0183 gear: bench-test with a
USB-RS485 adapter and confirm data flows in "send" mode and again in
"receive" mode. If it's backwards, flip `GWSERIAL_ELO` to `1`.

Known local build issue (unrelated to this board)
----------------------------------------------------
`pio run` currently fails to fetch the pinned
`ttlappalainen_NMEA2000=...NMEA2000.git#20251126` dependency on this machine
(git 2.50.1): PlatformIO fetches the tag into `FETCH_HEAD` but then runs
`git reset --hard 20251126` against a ref that was never created locally,
so it errors out. This reproduces identically on the stock
`m5stack-atoms3-canunit` environment, so it's a PlatformIO/git version
interaction, not something introduced by this board's files. Confirmed:
this environment resolves the board (`esp32-s3-devkitm-1`), installs the
ESP32-S3 toolchain, and gets to the same lib_deps step as every other
environment before hitting this wall.

Power / NMEA2000 LEN
---------------------
The CAN terminal here is a 2-pin CANH/CANL screw terminal with no bus-power
pin, and the board has its own isolated power supply - so it draws nothing
from the N2K bus. `N2K_LOAD_LEVEL` is left at `0` (the core's own default)
rather than the `3` (~150mA) used by boards that parasitically power
themselves from the bus.
