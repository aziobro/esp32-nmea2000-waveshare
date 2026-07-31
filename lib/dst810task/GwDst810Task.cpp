#include "GwDst810Task.h"
#include "GwHardware.h"

#ifndef GWDST810_ENABLE

void initDst810(GwApi *api)
{
    GwLog *logger = api->getLogger();
    LOG_DEBUG(GwLog::LOG, "no DST810 support for this board, task not started");
}

#else

#include "GWConfig.h"
#include "GwJsonDocument.h"
#include "GwSynchronized.h"
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
#include <NimBLEDevice.h>

// Nordic UART Service - the DST810 is the BLE peripheral/GATT server here
// (we're the central), so its "TX" characteristic (data FROM the sensor
// TO us) is the one we subscribe to for notifications.
static const char *NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

// Thread-safe handoff of a BLE address found by the scan callback (runs on
// NimBLE's own host task) to the main task loop (a different FreeRTOS
// task) - same GWSYNCHRONIZED mutex pattern used elsewhere in this project.
class Dst810FoundAddress
{
    SemaphoreHandle_t lock;
    bool found = false;
    NimBLEAddress address;

public:
    Dst810FoundAddress() { lock = xSemaphoreCreateMutex(); }
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

class Dst810ScanCallbacks : public NimBLEScanCallbacks
{
    Dst810FoundAddress *target;
    String namePrefix;
    String lockedAddress; // if non-empty, only this exact address matches - namePrefix is ignored
    GwLog *logger;

public:
    Dst810ScanCallbacks(Dst810FoundAddress *target, GwLog *logger) : target(target), logger(logger) {}
    void setNamePrefix(const String &prefix) { namePrefix = prefix; }
    void setLockedAddress(const String &addr) { lockedAddress = addr; }
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
    {
        String addr = advertisedDevice->getAddress().toString().c_str();
        if (lockedAddress.length() > 0)
        {
            // Locked to a specific, previously-learned sensor (see
            // dst810Addr) - ignore the name filter entirely, only this
            // exact address matches. Handles multiple DST810-like
            // sensors being in range at once (e.g. a neighboring boat's).
            if (!addr.equalsIgnoreCase(lockedAddress))
                return;
            LOG_DEBUG(GwLog::LOG, "DST810 scan matched locked address %s", addr.c_str());
            target->set(advertisedDevice->getAddress());
            return;
        }
        if (!advertisedDevice->haveName())
            return;
        String name = advertisedDevice->getName().c_str();
        if (namePrefix.length() > 0 && !name.startsWith(namePrefix))
            return;
        LOG_DEBUG(GwLog::LOG, "DST810 scan matched %s (%s)", name.c_str(), addr.c_str());
        target->set(advertisedDevice->getAddress());
    }
};

// Thread-safe accumulator for incoming BLE notification bytes, reassembled
// into complete NMEA0183 lines (a single sentence can span more than one
// BLE notification, depending on MTU). The notify callback runs on
// NimBLE's host task and only appends bytes here - kept minimal/fast,
// matching how BLE stacks expect notify callbacks to behave. The main
// task loop drains completed lines at its own pace.
class Dst810LineBuffer
{
    SemaphoreHandle_t lock;
    static const size_t BUFSIZE = 256;
    char buf[BUFSIZE];
    size_t len = 0;
    static const int MAXQUEUED = 8;
    String lines[MAXQUEUED];
    int qHead = 0, qCount = 0;

public:
    Dst810LineBuffer() { lock = xSemaphoreCreateMutex(); }
    void append(const uint8_t *data, size_t dataLen)
    {
        GWSYNCHRONIZED(lock);
        for (size_t i = 0; i < dataLen; i++)
        {
            char c = (char)data[i];
            if (c == '\r' || c == '\n')
            {
                if (len > 0)
                {
                    buf[len] = 0;
                    if (qCount < MAXQUEUED)
                    {
                        lines[(qHead + qCount) % MAXQUEUED] = String(buf);
                        qCount++;
                    }
                    // else: queue full - drop the line rather than block or
                    // grow unbounded; the main loop isn't keeping up, which
                    // shouldn't normally happen at NMEA0183 sentence rates.
                    len = 0;
                }
            }
            else if (len < BUFSIZE - 1)
            {
                buf[len++] = c;
            }
            else
            {
                // Overly long/garbled line - discard rather than overflow
                // or grow forever.
                len = 0;
            }
        }
    }
    bool getLine(String &out)
    {
        GWSYNCHRONIZED(lock);
        if (qCount == 0)
            return false;
        out = lines[qHead];
        qHead = (qHead + 1) % MAXQUEUED;
        qCount--;
        return true;
    }
};

// Thread-safe holder for the values our web request handler serves - same
// pattern as Icm20948WebData in lib/icm20948task/GwIcm20948Task.cpp.
class Dst810WebData
{
    SemaphoreHandle_t lock;
    bool connected = false;
    bool valid = false;
    double depthM = 0, offsetM = 0; // meters
    double speedMs = 0;             // m/s
    double tempC = 0;               // degrees C

public:
    Dst810WebData() { lock = xSemaphoreCreateMutex(); }
    void setConnected(bool c)
    {
        GWSYNCHRONIZED(lock);
        connected = c;
        if (!c)
            valid = false;
    }
    void setDepth(double depth, double offset)
    {
        GWSYNCHRONIZED(lock);
        depthM = depth;
        offsetM = offset;
        valid = true;
    }
    void setSpeed(double speed)
    {
        GWSYNCHRONIZED(lock);
        speedMs = speed;
        valid = true;
    }
    void setTemp(double t)
    {
        GWSYNCHRONIZED(lock);
        tempC = t;
        valid = true;
    }
    void toJson(GwJsonDocument &doc)
    {
        GWSYNCHRONIZED(lock);
        doc["connected"] = connected;
        doc["valid"] = valid;
        doc["depth"] = depthM;
        doc["offset"] = offsetM;
        doc["speed"] = speedMs;
        doc["tempC"] = tempC;
    }
};

static void processLine(const String &line, GwApi *api, GwConfigHandler *config, Dst810WebData &webData, unsigned char &sid)
{
    tNMEA0183Msg msg;
    if (!msg.SetMessage(line.c_str()))
        return;

    if (msg.IsMessageCode("DPT"))
    {
        double depth = 0, offset = 0, range = 0;
        if (!NMEA0183ParseDPT(msg, depth, offset, range))
            return;
        webData.setDepth(depth, offset);
        bool sendDep = true;
        config->getValue(sendDep, GwConfigDefinitions::dst810SendDep, true);
        if (sendDep)
        {
            tN2kMsg n2k;
            SetN2kWaterDepth(n2k, sid, depth, offset, range);
            api->sendN2kMessage(n2k);
            sid = (sid + 1) % 252;
        }
    }
    else if (msg.IsMessageCode("VHW"))
    {
        double trueHdg = 0, magHdg = 0, sow = 0;
        // NMEA0183ParseVHW already returns SOW in m/s (converted internally
        // from whichever of the knots/km-h fields is populated) - matches
        // SetN2kBoatSpeed's expected SI unit directly, no conversion needed.
        if (!NMEA0183ParseVHW(msg, trueHdg, magHdg, sow))
            return;
        webData.setSpeed(sow);
        bool sendSpd = true;
        config->getValue(sendSpd, GwConfigDefinitions::dst810SendSpd, true);
        if (sendSpd)
        {
            tN2kMsg n2k;
            SetN2kBoatSpeed(n2k, sid, sow, N2kDoubleNA, N2kSWRT_Ultra_Sound);
            api->sendN2kMessage(n2k);
            sid = (sid + 1) % 252;
        }
    }
    else if (msg.IsMessageCode("MTW"))
    {
        // No dedicated NMEA0183ParseMTW in the library - it's a trivial
        // single-value sentence ($--MTW,temp,C), read the raw field.
        double tempC = atof(msg.Field(0));
        webData.setTemp(tempC);
        bool sendTmp = true;
        config->getValue(sendTmp, GwConfigDefinitions::dst810SendTmp, true);
        if (sendTmp)
        {
            tN2kMsg n2k;
            SetN2kTemperatureExt(n2k, sid, 0, N2kts_SeaTemperature, CToKelvin(tempC));
            api->sendN2kMessage(n2k);
            sid = (sid + 1) % 252;
        }
    }
    // DST810 also streams $YXXDR (its own PTCH/ROLL attitude) and $VMVLW
    // (log distance) - not converted here. This unit already has a
    // dedicated, calibrated attitude source (lib/icm20948task); a second,
    // uncalibrated Attitude PGN from the same physical gateway would just
    // be confusing. VLW has no library parser and is a lower-value metric
    // than depth/speed/temp - can be added later if wanted.
}

static void runDst810Task(GwApi *api)
{
    GwLog *logger = api->getLogger();
    GwConfigHandler *config = api->getConfig();

    Dst810WebData webData;
    api->registerRequestHandler("data", [&webData](AsyncWebServerRequest *request)
                                {
        GwJsonDocument doc(160);
        webData.toJson(doc);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out); });

    String namePrefix = "DST";
    config->getValue(namePrefix, GwConfigDefinitions::dst810Name, "DST");
    String lockedAddress = "";
    config->getValue(lockedAddress, GwConfigDefinitions::dst810Addr, "");

    LOG_DEBUG(GwLog::LOG, "DST810 task starting, BLE name filter=%s, locked address=%s",
              namePrefix.c_str(), lockedAddress.length() > 0 ? lockedAddress.c_str() : "(none yet)");
    NimBLEDevice::init("");

    Dst810FoundAddress foundAddress;
    Dst810ScanCallbacks scanCallbacks(&foundAddress, logger);
    scanCallbacks.setNamePrefix(namePrefix);
    scanCallbacks.setLockedAddress(lockedAddress);

    NimBLEScan *pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(&scanCallbacks);
    // The DST810's name only appears in the scan RESPONSE (its main
    // advertisement is crowded with 128-bit service UUIDs) - passive
    // scanning would never see it. Bench-confirmed on a sibling project
    // (SailingComputer) integrating this same sensor - real bug, not a
    // guess.
    pScan->setActiveScan(true);

    NimBLEClient *pClient = nullptr;
    Dst810LineBuffer lineBuffer;
    unsigned char sid = 0;

    while (true)
    {
        bool enabled = true;
        config->getValue(enabled, GwConfigDefinitions::dst810Enable, true);
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
                LOG_DEBUG(GwLog::LOG, "DST810 connecting to %s", addr.toString().c_str());
                if (pClient->connect(addr))
                {
                    NimBLERemoteService *pService = pClient->getService(NUS_SERVICE_UUID);
                    NimBLERemoteCharacteristic *pChar = pService ? pService->getCharacteristic(NUS_TX_CHAR_UUID) : nullptr;
                    if (pChar && pChar->canNotify() &&
                        pChar->subscribe(true, [&lineBuffer](NimBLERemoteCharacteristic *, uint8_t *data, size_t len, bool)
                                          { lineBuffer.append(data, len); }))
                    {
                        LOG_DEBUG(GwLog::LOG, "DST810 connected and subscribed");
                        webData.setConnected(true);
                        // First time connecting (no address locked yet) -
                        // remember this specific sensor so future scans
                        // only ever match it, even if another DST810-like
                        // sensor comes into range later (e.g. a
                        // neighboring boat's). No restart needed - this
                        // only affects the NEXT scan/reconnect, and we're
                        // already connected for this session.
                        if (lockedAddress.length() == 0)
                        {
                            String newAddr = addr.toString().c_str();
                            config->updateValue(GwConfigDefinitions::dst810Addr, newAddr);
                            scanCallbacks.setLockedAddress(newAddr);
                            lockedAddress = newAddr;
                            LOG_DEBUG(GwLog::LOG, "DST810 locked to address %s for future scans", newAddr.c_str());
                        }
                    }
                    else
                    {
                        LOG_DEBUG(GwLog::ERROR, "DST810 connected but NUS service/characteristic not found - disconnecting");
                        pClient->disconnect();
                    }
                }
                else
                {
                    LOG_DEBUG(GwLog::ERROR, "DST810 connect failed, will retry");
                }
            }
            else if (!pScan->isScanning())
            {
                pScan->start(0);
            }
            delay(200);
            continue;
        }

        // Connected - drain any complete lines received since last loop.
        String line;
        while (lineBuffer.getLine(line))
        {
            processLine(line, api, config, webData, sid);
        }
        delay(50);
    }
}

void initDst810(GwApi *api)
{
    api->addCapability("dst810", "true");
    api->addUserTask(runDst810Task, String("dst810Task"), 6000);
}

#endif
