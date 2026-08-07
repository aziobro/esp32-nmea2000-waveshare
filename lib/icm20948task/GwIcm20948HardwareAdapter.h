#pragma once
#include "ImuTypes.h"
#include "GwLog.h"

/*
  Direct-register ICM-20948/AK09916 adapter. This is the only place in this
  task's own directory that touches the chip directly; everything it hands
  back feeds straight into the pure pipeline in lib/icm20948pure.
*/
class GwIcm20948HardwareAdapter
{
public:
    // Runs the full bring-up sequence: the established startup delay
    // (I2C/WiFi boot-race workaround), Wire.begin(), an I2C bus scan (for
    // diagnostics only), then repeated begin() attempts against both
    // possible AD0 addresses. Returns false if the sensor was never
    // found - callers should stop the task in that case, same as before.
    bool begin(GwLog *logger, int sdaPin, int sclPin);

    // Full-scale range - a one-time hardware setting applied once, same
    // as the pre-rewrite code (changing it requires a restart, handled
    // by the config system's own save-restarts-device behavior).
    void setFullScale(GwLog *logger, int accelRangeIdx, int gyrRangeIdx);

    bool dataReady();
    bool readAGMT(); // must be called once per cycle before the read*() functions below

    Vec3 readAccelG() const;      // g
    Vec3 readGyroDegPerSec() const; // degrees/second
    Vec3 readMagRaw() const;      // raw magnetometer units
    bool magValid() const { return lastMagValid; }

    // Lightweight recovery helpers used by the task loop before the
    // external watchdog escalates to a whole-device restart.
    bool reinitialize(GwLog *logger, int sdaPin, int sclPin);
    bool recoverMagBridge();

    uint32_t sensorErrorCount() const { return sensorErrors; }
    uint32_t i2cRecoveryCount() const { return i2cRecoveries; }
    uint32_t magRecoveryCount() const { return magRecoveries; }
    uint32_t magNoDataCount() const { return magNoData; }
    uint32_t magOverflowCount() const { return magOverflows; }
    uint32_t magZeroCount() const { return magZeroSamples; }
    uint32_t i2cRetryCount() const { return i2cRetries; }
    const char *lastStatusString() { return lastStatus; }

private:
    bool beginInternal(GwLog *logger, int sdaPin, int sclPin, bool startupDelay, bool scanBus);

    uint8_t imuAddress = 0;
    char lastStatus[64] = "direct backend not started";
    int activeSdaPin = -1;
    int activeSclPin = -1;
    bool beginDirect(uint8_t address);
    bool resetDirect();
    bool recoverDirectI2cBus(const char *reason);
    bool selectDirectBankStop(uint8_t bank);
    uint8_t endTransmissionWithRetry(uint8_t bank, uint8_t reg, const char *op, bool allowBusRecovery);
    bool writeDirectRegisterStop(uint8_t bank, uint8_t reg, uint8_t value);
    bool readDirectAgmtStop();
    bool readDirectRegistersStop(uint8_t bank, uint8_t reg, uint8_t *dst, uint8_t len);
    bool writeDirectAk09916Register8Stop(uint8_t reg, uint8_t value);
    bool readDirectAk09916Register8Stop(uint8_t reg, uint8_t &value);
    bool enableDirectMagDataReadStop(uint8_t reg, uint8_t bytes);
    bool waitDirectSlv4Stop(uint16_t timeoutMs);
    float directAccRangeFactor = 1.0f;
    float directGyrRangeFactor = 1.0f;
    double lastAccX = 0, lastAccY = 0, lastAccZ = 0;
    double lastGyrX = 0, lastGyrY = 0, lastGyrZ = 0;
    double lastMagX = 0, lastMagY = 0, lastMagZ = 0;
    double prevAccX = 0, prevAccY = 0, prevAccZ = 0;
    double prevGyrX = 0, prevGyrY = 0, prevGyrZ = 0;
    double prevMagX = 0, prevMagY = 0, prevMagZ = 0;
    bool lastMagValid = false;
    bool directMagStatusValid = false;
    bool havePreviousFrame = false;
    uint8_t repeatedFrameCount = 0;
    uint32_t sensorErrors = 0;
    uint32_t i2cRecoveries = 0;
    uint32_t magRecoveries = 0;
    uint32_t magNoData = 0;
    uint32_t magOverflows = 0;
    uint32_t magZeroSamples = 0;
    uint32_t i2cRetries = 0;
    uint8_t magRailCounts[3] = {0, 0, 0};
};
