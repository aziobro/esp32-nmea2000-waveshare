#include "GwCalypsoTask.h"
#include "GwHardware.h"

#ifndef GWCALYPSO_ENABLE

void initCalypso(GwApi *api)
{
    GwLog *logger = api->getLogger();
    LOG_DEBUG(GwLog::LOG, "no Calypso support for this board, task not started");
}

#else

#include "GWConfig.h"
#include "GwJsonDocument.h"
#include "GwSynchronized.h"
#include <N2kMessages.h>
#include <NimBLEDevice.h>

// Calypso Ultrasonic (LCJ Capteurs) wireless masthead wind sensor.
// Protocol per the maritime-labs/calypso-anemometer reference project
// (https://github.com/maritime-labs/calypso-anemometer, a Python/bleak
// project) - reverse-engineered from its source, not live-sniffed.
//
// Every unit advertises the identical literal name "ULTRASONIC" (no
// per-unit suffix like the DST810 has) and streams a 10-byte
// little-endian binary reading - not NMEA0183 text - over a notify
// characteristic aliased to the standard-form 16-bit UUID 0x2A39. No
// pairing, no GATT writes needed to start receiving data.
//
// The service UUID wrapping 0x2A39 is never published anywhere - the
// reference project's own BLE library resolves the characteristic
// without it - so we discover ALL primary services on connect and
// search each one's characteristics in turn until 0x2A39 turns up
// (unlike the DST810's single well-known Nordic UART Service).
//
// Reading layout, struct.unpack("<HHBBBBH", buf) in the reference decoder:
//   bytes 0-1  uint16  wind speed raw   -> speed_m_s   = raw / 100.0
//   bytes 2-3  uint16  wind angle raw   -> angle_deg   = raw (apparent, 0-359)
//   byte  4    uint8   battery raw      -> battery_pct = raw * 10
//   byte  5    uint8   temperature raw  -> temp_c      = raw - 100
//   byte  6    uint8   roll raw         -> roll_deg    = raw - 90
//   byte  7    uint8   pitch raw        -> pitch_deg   = raw - 90
//   bytes 8-9  uint16  heading raw      -> only meaningful with the
//                                          sensor's compass feature
//                                          separately enabled via a GATT
//                                          write this integration never
//                                          sends - not decoded here.
static const NimBLEUUID CALYPSO_READING_UUID((uint16_t)0x2A39);

// Same thread-safe scan-result handoff pattern as lib/dst810task -
// see that file for the full rationale.
class CalypsoFoundAddress
{
    SemaphoreHandle_t lock;
    bool found = false;
    NimBLEAddress address;

public:
    CalypsoFoundAddress() { lock = xSemaphoreCreateMutex(); }
    void set(const NimBLEAddress &addr)
    {
        GWSYNCHRONIZED(lock);
        address = addr;
        found = true;
    }
    bool take(NimBLEAddress &out)
    {
        GWSYNCHRONIZED(lock);
        if (!found)
            return false;
        out = address;
        found = false;
        return true;
    }
};

class CalypsoScanCallbacks : public NimBLEScanCallbacks
{
    CalypsoFoundAddress *target;
    String namePrefix;
    String lockedAddress; // if non-empty, only this exact address matches - namePrefix is ignored

public:
    CalypsoScanCallbacks(CalypsoFoundAddress *target) : target(target) {}
    void setNamePrefix(const String &prefix) { namePrefix = prefix; }
    void setLockedAddress(const String &addr) { lockedAddress = addr; }
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
    {
        String addr = advertisedDevice->getAddress().toString().c_str();
        if (lockedAddress.length() > 0)
        {
            // Locked to a specific, previously-learned sensor (see
            // calypsoAddr) - ignore the name filter entirely. Especially
            // important here since every Calypso advertises the exact
            // same name, so the name filter alone can never tell two
            // apart (e.g. a neighboring boat's).
            if (!addr.equalsIgnoreCase(lockedAddress))
                return;
            target->set(advertisedDevice->getAddress());
            return;
        }
        if (!advertisedDevice->haveName())
            return;
        String name = advertisedDevice->getName().c_str();
        if (namePrefix.length() > 0 && !name.startsWith(namePrefix))
            return;
        target->set(advertisedDevice->getAddress());
    }
};

// Thread-safe handoff of the latest decoded reading from the notify
// callback (runs on NimBLE's own host task) to the main task loop. A
// single "latest value" slot, not a queue like DST810's line buffer -
// each notification is a complete, self-contained snapshot, and if the
// main loop is a cycle behind it should process the newest reading, not
// replay every historical one in order.
class CalypsoReading
{
    SemaphoreHandle_t lock;
    bool hasNew = false;
    float windSpeedMs = 0, windAngleDeg = 0;
    int batteryPct = 0, tempC = 0, rollDeg = 0, pitchDeg = 0;

public:
    CalypsoReading() { lock = xSemaphoreCreateMutex(); }
    void set(float speedMs, float angleDeg, int batt, int temp, int roll, int pitch)
    {
        GWSYNCHRONIZED(lock);
        windSpeedMs = speedMs;
        windAngleDeg = angleDeg;
        batteryPct = batt;
        tempC = temp;
        rollDeg = roll;
        pitchDeg = pitch;
        hasNew = true;
    }
    bool take(float &speedMs, float &angleDeg, int &batt, int &temp, int &roll, int &pitch)
    {
        GWSYNCHRONIZED(lock);
        if (!hasNew)
            return false;
        speedMs = windSpeedMs;
        angleDeg = windAngleDeg;
        batt = batteryPct;
        temp = tempC;
        roll = rollDeg;
        pitch = pitchDeg;
        hasNew = false;
        return true;
    }
};

// Thread-safe holder for the values our web request handler serves -
// same pattern as Dst810WebData/Icm20948WebData.
class CalypsoWebData
{
    SemaphoreHandle_t lock;
    bool connected = false;
    bool valid = false;
    double windSpeedMs = 0, windAngleDeg = 0;
    int batteryPct = 0, tempC = 0, rollDeg = 0, pitchDeg = 0;

public:
    CalypsoWebData() { lock = xSemaphoreCreateMutex(); }
    void setConnected(bool c)
    {
        GWSYNCHRONIZED(lock);
        connected = c;
        if (!c)
            valid = false;
    }
    void update(double speedMs, double angleDeg, int batt, int temp, int roll, int pitch)
    {
        GWSYNCHRONIZED(lock);
        windSpeedMs = speedMs;
        windAngleDeg = angleDeg;
        batteryPct = batt;
        tempC = temp;
        rollDeg = roll;
        pitchDeg = pitch;
        valid = true;
    }
    void toJson(GwJsonDocument &doc)
    {
        GWSYNCHRONIZED(lock);
        doc["connected"] = connected;
        doc["valid"] = valid;
        doc["windSpeed"] = windSpeedMs;
        doc["windAngle"] = windAngleDeg;
        doc["battery"] = batteryPct;
        doc["tempC"] = tempC;
        doc["roll"] = rollDeg;
        doc["pitch"] = pitchDeg;
    }
};

static void runCalypsoTask(GwApi *api)
{
    GwLog *logger = api->getLogger();
    GwConfigHandler *config = api->getConfig();

    CalypsoWebData webData;
    api->registerRequestHandler("data", [&webData](AsyncWebServerRequest *request)
                                {
        GwJsonDocument doc(200);
        webData.toJson(doc);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });

    String namePrefix = "ULTRASONIC";
    config->getValue(namePrefix, GwConfigDefinitions::calypsoName, "ULTRASONIC");
    String lockedAddress = "";
    config->getValue(lockedAddress, GwConfigDefinitions::calypsoAddr, "");

    LOG_DEBUG(GwLog::LOG, "Calypso task starting, BLE name filter=%s, locked address=%s",
              namePrefix.c_str(), lockedAddress.length() > 0 ? lockedAddress.c_str() : "(none yet)");
    NimBLEDevice::init("");

    CalypsoFoundAddress foundAddress;
    CalypsoScanCallbacks scanCallbacks(&foundAddress);
    scanCallbacks.setNamePrefix(namePrefix);
    scanCallbacks.setLockedAddress(lockedAddress);

    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(&scanCallbacks);
    // Matches the DST810's finding that advertised names commonly live in
    // the scan response, not the primary advertisement - passive scanning
    // risks never seeing it.
    pScan->setActiveScan(true);

    NimBLEClient *pClient = nullptr;
    CalypsoReading reading;
    unsigned char sid = 0;

    while (true)
    {
        bool enabled = true;
        config->getValue(enabled, GwConfigDefinitions::calypsoEnable, true);
        if (!enabled)
        {
            if (pClient && pClient->isConnected())
                pClient->disconnect();
            webData.setConnected(false);
            if (pScan->isScanning())
                pScan->stop();
            delay(1000);
            continue;
        }

        if (!pClient || !pClient->isConnected())
        {
            webData.setConnected(false);
            NimBLEAddress addr;
            if (foundAddress.take(addr))
            {
                if (pScan->isScanning())
                    pScan->stop();
                if (!pClient)
                    pClient = NimBLEDevice::createClient();
                LOG_DEBUG(GwLog::LOG, "Calypso connecting to %s", addr.toString().c_str());
                if (pClient->connect(addr))
                {
                    NimBLERemoteCharacteristic *pChar = nullptr;
                    const std::vector<NimBLERemoteService *> &services = pClient->getServices(true);
                    for (auto *svc : services)
                    {
                        pChar = svc->getCharacteristic(CALYPSO_READING_UUID);
                        if (pChar)
                            break;
                    }
                    if (pChar && pChar->canNotify() &&
                        pChar->subscribe(true, [&reading](NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool)
                                          {
                            if (len < 10) return;
                            uint16_t speedRaw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
                            uint16_t angleRaw = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
                            uint8_t battRaw = data[4];
                            uint8_t tempRaw = data[5];
                            uint8_t rollRaw = data[6];
                            uint8_t pitchRaw = data[7];
                            reading.set(speedRaw / 100.0f, (float)angleRaw,
                                        (int)battRaw * 10, (int)tempRaw - 100,
                                        (int)rollRaw - 90, (int)pitchRaw - 90); }))
                    {
                        LOG_DEBUG(GwLog::LOG, "Calypso connected and subscribed");
                        webData.setConnected(true);
                        // First time connecting (no address locked yet) -
                        // remember this specific sensor so future scans
                        // only ever match it. Especially important here
                        // since every Calypso-like sensor in range (e.g. a
                        // neighboring boat's) advertises the same name.
                        if (lockedAddress.length() == 0)
                        {
                            String newAddr = addr.toString().c_str();
                            config->updateValue(GwConfigDefinitions::calypsoAddr, newAddr);
                            scanCallbacks.setLockedAddress(newAddr);
                            lockedAddress = newAddr;
                            LOG_DEBUG(GwLog::LOG, "Calypso locked to address %s for future scans", newAddr.c_str());
                        }
                    }
                    else
                    {
                        LOG_DEBUG(GwLog::ERROR, "Calypso connected but reading characteristic (0x2A39) not found in any service - disconnecting");
                        pClient->disconnect();
                    }
                }
                else
                {
                    LOG_DEBUG(GwLog::ERROR, "Calypso connect failed, will retry");
                }
            }
            else if (!pScan->isScanning())
            {
                pScan->start(0);
            }
            delay(200);
            continue;
        }

        // Connected - process the latest reading, if a new one has
        // arrived since last loop.
        float speedMs, angleDeg;
        int batt, temp, roll, pitch;
        if (reading.take(speedMs, angleDeg, batt, temp, roll, pitch))
        {
            webData.update(speedMs, angleDeg, batt, temp, roll, pitch);
            bool sendWind = true;
            config->getValue(sendWind, GwConfigDefinitions::calypsoSendWind, true);
            if (sendWind)
            {
                tN2kMsg n2k;
                SetN2kWindSpeed(n2k, sid, speedMs, DegToRad(angleDeg), N2kWind_Apparent);
                api->sendN2kMessage(n2k);
                sid = (sid + 1) % 252;
            }
        }
        delay(50);
    }
}

void initCalypso(GwApi *api)
{
    api->addCapability("calypso", "true");
    api->addUserTask(runCalypsoTask, String("calypsoTask"), 6000);
}

#endif
