#include "GwIcm20948HardwareAdapter.h"
#include <Wire.h>
#include <math.h>
#include <stdio.h>

static const uint32_t ICM20948_I2C_CLOCK_HZ = 50000;
static const uint16_t ICM20948_I2C_TIMEOUT_MS = 250;
static const uint8_t ICM20948_I2C_RETRY_ATTEMPTS = 3;
static const uint16_t ICM20948_I2C_RETRY_DELAY_MS = 2;
static const int ICM20948_DMP_ODR_INTERVAL = 4; // DMP 55Hz / (4 + 1) = 11Hz
static const double ICM20948_AGMT_MAG_RAIL_UT = 38.4;
static const double ICM20948_AGMT_MAG_RAIL_TOLERANCE_UT = 0.03;
static const uint8_t ICM20948_AGMT_MAG_RAIL_REJECT_SAMPLES = 3;
static const uint8_t ICM20948_IDENTICAL_FRAME_REJECT_SAMPLES = 5;
static const uint8_t ICM20948_REG_BANK_SEL = 0x7F;
static const uint8_t ICM20948_WHO_AM_I = 0x00;
static const uint8_t ICM20948_WHO_AM_I_CONTENT = 0xEA;
static const uint8_t ICM20948_USER_CTRL = 0x03;
static const uint8_t ICM20948_PWR_MGMT_1 = 0x06;
static const uint8_t ICM20948_ACCEL_OUT = 0x2D;
static const uint8_t ICM20948_ODR_ALIGN_EN = 0x09;
static const uint8_t ICM20948_GYRO_CONFIG_1 = 0x01;
static const uint8_t ICM20948_ACCEL_CONFIG = 0x14;
static const uint8_t ICM20948_I2C_MST_CTRL = 0x01;
static const uint8_t ICM20948_I2C_SLV0_ADDR = 0x03;
static const uint8_t ICM20948_I2C_SLV0_REG = 0x04;
static const uint8_t ICM20948_I2C_SLV0_CTRL = 0x05;
static const uint8_t ICM20948_I2C_SLV4_ADDR = 0x13;
static const uint8_t ICM20948_I2C_SLV4_REG = 0x14;
static const uint8_t ICM20948_I2C_SLV4_CTRL = 0x15;
static const uint8_t ICM20948_I2C_SLV4_DO = 0x16;
static const uint8_t ICM20948_I2C_SLV4_DI = 0x17;
static const uint8_t ICM20948_RESET = 0x80;
static const uint8_t ICM20948_I2C_MST_EN = 0x20;
static const uint8_t ICM20948_I2C_MST_RST = 0x02;
static const uint8_t ICM20948_I2C_SLVX_EN = 0x80;
static const uint8_t AK09916_ADDRESS = 0x0C;
static const uint8_t AK09916_READ = 0x80;
static const uint8_t AK09916_WIA_1 = 0x00;
static const uint8_t AK09916_WIA_2 = 0x01;
static const uint8_t AK09916_STATUS_1 = 0x10;
static const uint8_t AK09916_HXL = 0x11;
static const uint8_t AK09916_CNTL_2 = 0x31;
static const uint8_t AK09916_CNTL_3 = 0x32;
static const uint8_t AK09916_CONT_MODE_100HZ = 0x08;
static const double ICM20948_AK09916_MAG_LSB_UT = 0.1495;

static bool isNearAbs(double value, double target, double tolerance)
{
    return fabs(fabs(value) - target) <= tolerance;
}

bool GwIcm20948HardwareAdapter::begin(GwLog *logger, int sdaPin, int sclPin)
{
    return beginInternal(logger, sdaPin, sclPin, true, true);
}

bool GwIcm20948HardwareAdapter::beginInternal(GwLog *logger, int sdaPin, int sclPin, bool startupDelay, bool scanBus)
{
    // Real, bench-confirmed bug (not hypothetical): starting I2C this
    // early intermittently (roughly 1 boot in 3-4, confirmed via repeated
    // hard-reset soak tests) corrupts memory badly enough to crash a
    // completely unrelated task several hundred ms later (Guru
    // Meditation, stack canary tripped on IDLE1) - NOT a stack-size
    // problem. The timing lines up almost exactly with WiFi/RF init
    // finishing in the boot log, so I2C bring-up is pushed a couple of
    // seconds later than the framework would otherwise start it, past
    // that contention window, rather than racing it. Verified: 9 crashes
    // in 32 cold boots without this delay, 0 crashes in 24 cold boots
    // with it. Unchanged from the pre-rewrite code - do not remove.
    if (startupDelay)
        delay(2500);
#if defined(GWICM20948_USE_WOLLEWALD)
    activeSdaPin = sdaPin;
    activeSclPin = sclPin;
#endif
    Wire.setTimeOut(ICM20948_I2C_TIMEOUT_MS); // bound any I2C hang instead of blocking this task forever
    bool wireOk = Wire.begin(sdaPin, sclPin);
    Wire.setClock(ICM20948_I2C_CLOCK_HZ);
    LOG_DEBUG(GwLog::LOG, "Wire.begin returned %d, clock=%luHz timeout=%ums",
              (int)wireOk, (unsigned long)ICM20948_I2C_CLOCK_HZ, (unsigned int)ICM20948_I2C_TIMEOUT_MS);

    if (scanBus)
    {
        LOG_DEBUG(GwLog::LOG, "scanning I2C bus...");
        int devicesFound = 0;
        for (uint8_t addr = 1; addr < 127; addr++)
        {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0)
            {
                LOG_DEBUG(GwLog::LOG, "  I2C device found at 0x%02X", addr);
                devicesFound++;
            }
        }
        LOG_DEBUG(GwLog::LOG, "I2C scan done, %d device(s) found", devicesFound);
    }

    bool found = false;
#if defined(GWICM20948_USE_WOLLEWALD)
    imuAddress = 0;
    const uint8_t addresses[2] = {0x69, 0x68};
    for (int attempt = 0; attempt < 6 && !found; attempt++)
    {
        int idx = attempt % 2;
        LOG_DEBUG(GwLog::LOG, "ICM20948 direct Wollewald-compatible init attempt %d (addr=0x%02X) starting",
                  attempt, addresses[idx]);
        if (beginWollewaldDirect(addresses[idx]))
        {
            found = true;
            snprintf(lastStatus, sizeof(lastStatus), "direct init ok addr=0x%02X", addresses[idx]);
            LOG_DEBUG(GwLog::LOG, "ICM20948 direct init attempt %d (addr=0x%02X) result: ok",
                      attempt, addresses[idx]);
        }
        else
        {
            LOG_DEBUG(GwLog::LOG, "ICM20948 direct init attempt %d (addr=0x%02X) result: failed (%s)",
                      attempt, addresses[idx], lastStatus);
            imuAddress = 0;
            delay(300);
        }
    }
#else
    // AD0_VAL=1 -> 0x69 (default, ADR jumper open), AD0_VAL=0 -> 0x68 (jumper closed)
    for (int attempt = 0; attempt < 6 && !found; attempt++)
    {
        uint8_t ad0 = (attempt % 2 == 0) ? 1 : 0;
        LOG_DEBUG(GwLog::LOG, "ICM20948 init attempt %d (AD0=%d) starting", attempt, ad0);
        imu.begin(Wire, ad0);
        LOG_DEBUG(GwLog::LOG, "ICM20948 init attempt %d (AD0=%d) result: %s", attempt, ad0, imu.statusString());
        if (imu.status == ICM_20948_Stat_Ok)
            found = true;
        else
            delay(300);
    }
#endif
    if (!found)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 not found on sda=%d,scl=%d", sdaPin, sclPin);
        return false;
    }
    LOG_DEBUG(GwLog::LOG, "ICM20948 found");
    return true;
}

bool GwIcm20948HardwareAdapter::reinitialize(GwLog *logger, int sdaPin, int sclPin)
{
    LOG_DEBUG(GwLog::ERROR, "ICM20948 runtime reinitialize starting");
    Wire.end();
    delay(25);
    havePreviousFrame = false;
    repeatedFrameCount = 0;
    magRailCounts[0] = 0;
    magRailCounts[1] = 0;
    magRailCounts[2] = 0;
    return beginInternal(logger, sdaPin, sclPin, false, false);
}

void GwIcm20948HardwareAdapter::setFullScale(GwLog *logger, int accelRangeIdx, int gyrRangeIdx)
{
#if defined(GWICM20948_USE_WOLLEWALD)
    if (imuAddress == 0)
        return;
    if (accelRangeIdx < 0)
        accelRangeIdx = 0;
    if (accelRangeIdx > 3)
        accelRangeIdx = 3;
    if (gyrRangeIdx < 0)
        gyrRangeIdx = 0;
    if (gyrRangeIdx > 3)
        gyrRangeIdx = 3;
    uint8_t accCfg = 0;
    if (readWollewaldRegistersStop(2, ICM20948_ACCEL_CONFIG, &accCfg, 1))
    {
        accCfg &= ~(0x06);
        accCfg |= (uint8_t)(accelRangeIdx << 1);
        writeWollewaldRegisterStop(2, ICM20948_ACCEL_CONFIG, accCfg);
    }
    uint8_t gyrCfg = 0;
    if (readWollewaldRegistersStop(2, ICM20948_GYRO_CONFIG_1, &gyrCfg, 1))
    {
        gyrCfg &= ~(0x06);
        gyrCfg |= (uint8_t)(gyrRangeIdx << 1);
        writeWollewaldRegisterStop(2, ICM20948_GYRO_CONFIG_1, gyrCfg);
    }
    wollewaldAccRangeFactor = (float)(1 << accelRangeIdx);
    wollewaldGyrRangeFactor = (float)(1 << gyrRangeIdx);
    LOG_DEBUG(GwLog::LOG, "ICM20948 direct setFullScale acc=%d gyr=%d", accelRangeIdx, gyrRangeIdx);
#else
    ICM_20948_fss_t fss;
    fss.a = (uint8_t)accelRangeIdx;
    fss.g = (uint8_t)gyrRangeIdx;
    imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
    LOG_DEBUG(GwLog::LOG, "ICM20948 setFullScale acc=%d gyr=%d result: %s", accelRangeIdx, gyrRangeIdx, imu.statusString());
#endif
}

bool GwIcm20948HardwareAdapter::initDmp(GwLog *logger)
{
#if defined(GWICM20948_USE_WOLLEWALD)
    (void)logger;
    snprintf(lastStatus, sizeof(lastStatus), "Wollewald raw-only backend");
    return false;
#else
    ICM_20948_Status_e stat = imu.initializeDMP();
    if (stat != ICM_20948_Stat_Ok)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 DMP initializeDMP failed: %s", imu.statusString());
        sensorErrors++;
        return false;
    }
    stat = imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);
    if (stat != ICM_20948_Stat_Ok)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 DMP orientation enable failed: %s", imu.statusString());
        sensorErrors++;
        return false;
    }
    // 32-bit calibrated compass (Compass_Calibr FIFO field) - the DMP's own
    // magnetometer path, reconfigures I2C_SLV0 for its own reverse-engineered
    // shadow-register layout as a side effect of initializeDMP() above. That
    // reconfiguration is exactly why the plain getAGMT()/readMagRaw() path
    // (which assumes the non-DMP layout) silently decodes garbage once DMP
    // is active - see doc/IcmMagnetometerDmpConflict.md. Reading the
    // magnetometer via this DMP-native field instead sidesteps the conflict
    // entirely rather than trying to keep two I2C_SLV0 consumers in sync.
    stat = imu.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD);
    if (stat != ICM_20948_Stat_Ok)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 DMP geomagnetic enable failed: %s", imu.statusString());
        sensorErrors++;
        return false;
    }
    stat = imu.setDMPODRrate(DMP_ODR_Reg_Quat9, ICM20948_DMP_ODR_INTERVAL);
    if (stat != ICM_20948_Stat_Ok)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 DMP Quat9 ODR failed: %s", imu.statusString());
        sensorErrors++;
        return false;
    }
    stat = imu.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, ICM20948_DMP_ODR_INTERVAL);
    if (stat != ICM_20948_Stat_Ok)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 DMP compass ODR failed: %s", imu.statusString());
        sensorErrors++;
        return false;
    }
    stat = imu.enableFIFO();
    if (stat != ICM_20948_Stat_Ok)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 DMP enableFIFO failed: %s", imu.statusString());
        sensorErrors++;
        return false;
    }
    stat = imu.enableDMP();
    if (stat != ICM_20948_Stat_Ok)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 DMP enableDMP failed: %s", imu.statusString());
        sensorErrors++;
        return false;
    }
    stat = imu.resetDMP();
    if (stat != ICM_20948_Stat_Ok)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 DMP resetDMP failed: %s", imu.statusString());
        sensorErrors++;
        return false;
    }
    stat = imu.resetFIFO();
    if (stat != ICM_20948_Stat_Ok)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 DMP resetFIFO failed: %s", imu.statusString());
        sensorErrors++;
        return false;
    }
    LOG_DEBUG(GwLog::LOG, "ICM20948 DMP init ok");
    return true;
#endif
}

bool GwIcm20948HardwareAdapter::dataReady()
{
#if defined(GWICM20948_USE_WOLLEWALD)
    return imuAddress != 0;
#else
    return imu.dataReady();
#endif
}

#if defined(GWICM20948_USE_WOLLEWALD)
bool GwIcm20948HardwareAdapter::writeWollewaldRegisterStop(uint8_t bank, uint8_t reg, uint8_t value)
{
    if (imuAddress == 0)
        return false;

    if (!selectWollewaldBankStop(bank))
        return false;

    uint8_t err = 0;
    for (uint8_t attempt = 0; attempt < ICM20948_I2C_RETRY_ATTEMPTS; attempt++)
    {
        Wire.beginTransmission(imuAddress);
        Wire.write(reg);
        Wire.write(value);
        err = Wire.endTransmission(true);
        if (err == 0)
            break;
        if (attempt + 1 < ICM20948_I2C_RETRY_ATTEMPTS)
        {
            i2cRetries++;
            delay(ICM20948_I2C_RETRY_DELAY_MS);
        }
    }
    if (err != 0)
    {
        sensorErrors++;
        snprintf(lastStatus, sizeof(lastStatus), "I2C write b%u r%02X err=%u",
                 (unsigned int)bank, (unsigned int)reg, (unsigned int)err);
        recoverWollewaldI2cBus("write");
        return false;
    }
    delayMicroseconds(50);
    return true;
}

bool GwIcm20948HardwareAdapter::writeWollewaldRegister16Stop(uint8_t bank, uint8_t reg, uint16_t value)
{
    if (imuAddress == 0)
        return false;

    if (!selectWollewaldBankStop(bank))
        return false;

    uint8_t err = 0;
    for (uint8_t attempt = 0; attempt < ICM20948_I2C_RETRY_ATTEMPTS; attempt++)
    {
        Wire.beginTransmission(imuAddress);
        Wire.write(reg);
        Wire.write((uint8_t)(value >> 8));
        Wire.write((uint8_t)value);
        err = Wire.endTransmission(true);
        if (err == 0)
            break;
        if (attempt + 1 < ICM20948_I2C_RETRY_ATTEMPTS)
        {
            i2cRetries++;
            delay(ICM20948_I2C_RETRY_DELAY_MS);
        }
    }
    if (err != 0)
    {
        sensorErrors++;
        snprintf(lastStatus, sizeof(lastStatus), "I2C write16 b%u r%02X err=%u",
                 (unsigned int)bank, (unsigned int)reg, (unsigned int)err);
        recoverWollewaldI2cBus("write16");
        return false;
    }
    delayMicroseconds(50);
    return true;
}

bool GwIcm20948HardwareAdapter::readWollewaldRegistersStop(uint8_t bank, uint8_t reg, uint8_t *dst, uint8_t len)
{
    if (imuAddress == 0 || dst == nullptr || len == 0)
        return false;

    if (!selectWollewaldBankStop(bank))
        return false;

    Wire.beginTransmission(imuAddress);
    Wire.write(reg);
    uint8_t err = endTransmissionWithRetry(bank, reg, "reg select", true);
    if (err != 0)
        return false;
    delayMicroseconds(50);

    uint8_t got = 0;
    for (uint8_t attempt = 0; attempt < ICM20948_I2C_RETRY_ATTEMPTS; attempt++)
    {
        got = Wire.requestFrom(imuAddress, len, (uint8_t)true);
        if (got == len)
            break;
        while (Wire.available())
            Wire.read();
        if (attempt + 1 < ICM20948_I2C_RETRY_ATTEMPTS)
        {
            i2cRetries++;
            delay(ICM20948_I2C_RETRY_DELAY_MS);
            selectWollewaldBankStop(bank);
            Wire.beginTransmission(imuAddress);
            Wire.write(reg);
            if (endTransmissionWithRetry(bank, reg, "reg reselect", false) != 0)
                break;
        }
    }
    if (got != len)
    {
        while (Wire.available())
            Wire.read();
        sensorErrors++;
        snprintf(lastStatus, sizeof(lastStatus), "I2C short read %u/%u", (unsigned int)got, (unsigned int)len);
        recoverWollewaldI2cBus("short read");
        return false;
    }

    for (uint8_t i = 0; i < len; i++)
        dst[i] = Wire.read();
    return true;
}

bool GwIcm20948HardwareAdapter::recoverWollewaldI2cBus(const char *reason)
{
    if (activeSdaPin < 0 || activeSclPin < 0)
        return false;

    i2cRecoveries++;
    Wire.end();
    delay(5);
    Wire.setTimeOut(ICM20948_I2C_TIMEOUT_MS);
    bool ok = Wire.begin(activeSdaPin, activeSclPin);
    Wire.setClock(ICM20948_I2C_CLOCK_HZ);
    snprintf(lastStatus, sizeof(lastStatus), "I2C recover %s %s",
             reason ? reason : "", ok ? "ok" : "failed");
    delay(2);
    return ok;
}

bool GwIcm20948HardwareAdapter::selectWollewaldBankStop(uint8_t bank)
{
    if (imuAddress == 0)
        return false;

    Wire.beginTransmission(imuAddress);
    Wire.write(ICM20948_REG_BANK_SEL);
    Wire.write(bank << 4);
    uint8_t err = endTransmissionWithRetry(bank, ICM20948_REG_BANK_SEL, "bank", true);
    if (err != 0)
        return false;
    delayMicroseconds(50);
    return true;
}

uint8_t GwIcm20948HardwareAdapter::endTransmissionWithRetry(uint8_t bank, uint8_t reg, const char *op, bool allowBusRecovery)
{
    uint8_t err = Wire.endTransmission(true);
    if (err == 0)
        return 0;

    for (uint8_t attempt = 1; attempt < ICM20948_I2C_RETRY_ATTEMPTS; attempt++)
    {
        i2cRetries++;
        delay(ICM20948_I2C_RETRY_DELAY_MS);
        Wire.beginTransmission(imuAddress);
        if (reg == ICM20948_REG_BANK_SEL && op && op[0] == 'b')
        {
            Wire.write(ICM20948_REG_BANK_SEL);
            Wire.write(bank << 4);
        }
        else
        {
            Wire.write(reg);
        }
        err = Wire.endTransmission(true);
        if (err == 0)
            return 0;
    }

    sensorErrors++;
    snprintf(lastStatus, sizeof(lastStatus), "I2C %s b%u r%02X err=%u",
             op ? op : "tx", (unsigned int)bank, (unsigned int)reg, (unsigned int)err);
    if (allowBusRecovery)
        recoverWollewaldI2cBus(op);
    return err;
}

bool GwIcm20948HardwareAdapter::waitWollewaldSlv4Stop(uint16_t timeoutMs)
{
    uint32_t start = millis();
    do
    {
        uint8_t ctrl = 0;
        if (!readWollewaldRegistersStop(3, ICM20948_I2C_SLV4_CTRL, &ctrl, 1))
            return false;
        if ((ctrl & ICM20948_I2C_SLVX_EN) == 0)
            return true;
        delay(1);
    } while ((uint32_t)(millis() - start) < timeoutMs);

    // The AK09916 is reached through the ICM-20948's internal I2C master.
    // Wollewald's driver bounds this wait but does not treat timeout as
    // fatal; doing so here caused repeated full IMU reinitializations during
    // tumble captures even while subsequent shadow-register samples were
    // usable. Record the slow bridge transaction, then continue.
    sensorErrors++;
    snprintf(lastStatus, sizeof(lastStatus), "AK09916 SLV4 slow");
    return true;
}

bool GwIcm20948HardwareAdapter::writeWollewaldAk09916Register8Stop(uint8_t reg, uint8_t value)
{
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV4_ADDR, AK09916_ADDRESS))
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV4_DO, value))
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV4_REG, reg))
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVX_EN))
        return false;
    return waitWollewaldSlv4Stop(100);
}

bool GwIcm20948HardwareAdapter::readWollewaldAk09916Register8Stop(uint8_t reg, uint8_t &value)
{
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV4_ADDR, AK09916_ADDRESS | AK09916_READ))
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV4_REG, reg))
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVX_EN))
        return false;
    if (!waitWollewaldSlv4Stop(100))
        return false;
    return readWollewaldRegistersStop(3, ICM20948_I2C_SLV4_DI, &value, 1);
}

bool GwIcm20948HardwareAdapter::enableWollewaldMagDataReadStop(uint8_t reg, uint8_t bytes)
{
    if (bytes == 0 || bytes > 15)
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV0_CTRL, 0))
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | AK09916_READ))
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV0_REG, reg))
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV0_CTRL, ICM20948_I2C_SLVX_EN | bytes))
        return false;
    delay(10);
    return true;
}

bool GwIcm20948HardwareAdapter::recoverMagBridge()
{
    if (imuAddress == 0)
        return false;
    magRecoveries++;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_SLV0_CTRL, 0))
        return false;
    if (!writeWollewaldRegisterStop(0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN | ICM20948_I2C_MST_RST))
        return false;
    delay(10);
    if (!writeWollewaldRegisterStop(0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN))
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_MST_CTRL, 0x07))
        return false;
    if (!writeWollewaldAk09916Register8Stop(AK09916_CNTL_3, 0x01))
        return false;
    delay(100);
    if (!writeWollewaldAk09916Register8Stop(AK09916_CNTL_2, AK09916_CONT_MODE_100HZ))
        return false;
    if (!enableWollewaldMagDataReadStop(AK09916_HXL, 0x08))
        return false;
    snprintf(lastStatus, sizeof(lastStatus), "AK09916 bridge recovered");
    return true;
}

bool GwIcm20948HardwareAdapter::resetWollewaldDirect()
{
    if (!writeWollewaldRegisterStop(0, ICM20948_PWR_MGMT_1, ICM20948_RESET))
        return false;
    delay(100);
    return true;
}

bool GwIcm20948HardwareAdapter::beginWollewaldDirect(uint8_t address)
{
    imuAddress = address;
    snprintf(lastStatus, sizeof(lastStatus), "direct init probing 0x%02X", (unsigned int)address);

    if (!resetWollewaldDirect())
        return false;

    uint8_t who = 0;
    bool whoOk = false;
    for (uint8_t tries = 0; tries < 3 && !whoOk; tries++)
    {
        if (readWollewaldRegistersStop(0, ICM20948_WHO_AM_I, &who, 1) &&
            who == ICM20948_WHO_AM_I_CONTENT)
        {
            whoOk = true;
            break;
        }
        delay(10);
    }
    if (!whoOk)
    {
        snprintf(lastStatus, sizeof(lastStatus), "WHO_AM_I 0x%02X at 0x%02X",
                 (unsigned int)who, (unsigned int)address);
        return false;
    }

    if (!writeWollewaldRegisterStop(0, ICM20948_PWR_MGMT_1, 0x01))
        return false;
    delay(10);
    if (!writeWollewaldRegisterStop(2, ICM20948_ODR_ALIGN_EN, 1))
        return false;
    if (!writeWollewaldRegisterStop(0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN))
        return false;
    if (!writeWollewaldRegisterStop(3, ICM20948_I2C_MST_CTRL, 0x07))
        return false;
    delay(10);

    if (!writeWollewaldAk09916Register8Stop(AK09916_CNTL_3, 0x01))
        return false;
    delay(100);

    bool magOk = false;
    for (uint8_t tries = 0; tries < 10 && !magOk; tries++)
    {
        if (!writeWollewaldRegisterStop(0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN))
            return false;
        if (!writeWollewaldRegisterStop(3, ICM20948_I2C_MST_CTRL, 0x07))
            return false;
        delay(10);

        uint8_t msb = 0;
        uint8_t lsb = 0;
        if (readWollewaldAk09916Register8Stop(AK09916_WIA_1, msb) &&
            readWollewaldAk09916Register8Stop(AK09916_WIA_2, lsb))
        {
            uint16_t magId = ((uint16_t)msb << 8) | lsb;
            if (magId == 0x4809 || magId == 0x0948)
            {
                magOk = true;
                break;
            }
            snprintf(lastStatus, sizeof(lastStatus), "AK09916 id 0x%04X", (unsigned int)magId);
        }

        uint8_t userCtrl = 0;
        if (readWollewaldRegistersStop(0, ICM20948_USER_CTRL, &userCtrl, 1))
        {
            writeWollewaldRegisterStop(0, ICM20948_USER_CTRL, userCtrl | ICM20948_I2C_MST_RST);
            delay(10);
        }
    }
    if (!magOk)
        return false;

    if (!writeWollewaldAk09916Register8Stop(AK09916_CNTL_2, AK09916_CONT_MODE_100HZ))
        return false;
    delay(10);
    if (!enableWollewaldMagDataReadStop(AK09916_HXL, 0x08))
        return false;

    snprintf(lastStatus, sizeof(lastStatus), "direct init ok addr=0x%02X", (unsigned int)address);
    return true;
}

bool GwIcm20948HardwareAdapter::readWollewaldAgmtStop()
{
    uint8_t buffer[22] = {0};
    if (!readWollewaldRegistersStop(0, ICM20948_ACCEL_OUT, buffer, sizeof(buffer)))
        return false;
    wollewaldMagStatusValid = false;

    int16_t accX = static_cast<int16_t>(((uint16_t)buffer[0] << 8) | buffer[1]);
    int16_t accY = static_cast<int16_t>(((uint16_t)buffer[2] << 8) | buffer[3]);
    int16_t accZ = static_cast<int16_t>(((uint16_t)buffer[4] << 8) | buffer[5]);
    int16_t gyrX = static_cast<int16_t>(((uint16_t)buffer[6] << 8) | buffer[7]);
    int16_t gyrY = static_cast<int16_t>(((uint16_t)buffer[8] << 8) | buffer[9]);
    int16_t gyrZ = static_cast<int16_t>(((uint16_t)buffer[10] << 8) | buffer[11]);
    int16_t magX = static_cast<int16_t>(((uint16_t)buffer[15] << 8) | buffer[14]);
    int16_t magY = static_cast<int16_t>(((uint16_t)buffer[17] << 8) | buffer[16]);
    int16_t magZ = static_cast<int16_t>(((uint16_t)buffer[19] << 8) | buffer[18]);
    uint8_t magSt2 = buffer[21];

    lastAccX = ((double)accX) * wollewaldAccRangeFactor / 16384.0;
    lastAccY = ((double)accY) * wollewaldAccRangeFactor / 16384.0;
    lastAccZ = ((double)accZ) * wollewaldAccRangeFactor / 16384.0;
    lastGyrX = ((double)gyrX) * wollewaldGyrRangeFactor * 250.0 / 32768.0;
    lastGyrY = ((double)gyrY) * wollewaldGyrRangeFactor * 250.0 / 32768.0;
    lastGyrZ = ((double)gyrZ) * wollewaldGyrRangeFactor * 250.0 / 32768.0;
    if ((magSt2 & 0x08) != 0)
    {
        magOverflows++;
        lastMagValid = false;
        snprintf(lastStatus, sizeof(lastStatus), "AK09916 overflow");
        return true;
    }
    if (magX == 0 && magY == 0 && magZ == 0)
    {
        magZeroSamples++;
        lastMagValid = false;
        snprintf(lastStatus, sizeof(lastStatus), "AK09916 zero sample");
        return true;
    }

    lastMagX = ((double)magX) * ICM20948_AK09916_MAG_LSB_UT;
    lastMagY = ((double)magY) * ICM20948_AK09916_MAG_LSB_UT;
    lastMagZ = ((double)magZ) * ICM20948_AK09916_MAG_LSB_UT;
    wollewaldMagStatusValid = true;
    lastMagValid = true;
    return true;
}
#endif

bool GwIcm20948HardwareAdapter::readAGMT()
{
#if defined(GWICM20948_USE_WOLLEWALD)
    if (imuAddress == 0)
    {
        snprintf(lastStatus, sizeof(lastStatus), "Wollewald no active device");
        sensorErrors++;
        lastMagValid = false;
        return false;
    }
    if (!readWollewaldAgmtStop())
        return false;
#else
    imu.getAGMT();
    // Always copy out the fields the library decoded. On real hardware we
    // have seen getAGMT() return Data Underflow while accel/gyro are sane but
    // the embedded AK09916 mag path is absent/zero; that should invalidate
    // heading, not make the whole IMU task reinitialize in a tight loop.
    lastAccX = imu.accX() / 1000.0;
    lastAccY = imu.accY() / 1000.0;
    lastAccZ = imu.accZ() / 1000.0;
    lastGyrX = imu.gyrX();
    lastGyrY = imu.gyrY();
    lastGyrZ = imu.gyrZ();
    lastMagX = imu.magX();
    lastMagY = imu.magY();
    lastMagZ = imu.magZ();
#endif

    double accNorm = sqrt(lastAccX * lastAccX + lastAccY * lastAccY + lastAccZ * lastAccZ);
    bool accelPlausible = isfinite(accNorm) && accNorm > 0.2 && accNorm < 2.5;
    bool gyroPlausible = isfinite(lastGyrX) && isfinite(lastGyrY) && isfinite(lastGyrZ);
    double magNorm = sqrt(lastMagX * lastMagX + lastMagY * lastMagY + lastMagZ * lastMagZ);

    bool identicalFrame = havePreviousFrame &&
                          lastAccX == prevAccX && lastAccY == prevAccY && lastAccZ == prevAccZ &&
                          lastGyrX == prevGyrX && lastGyrY == prevGyrY && lastGyrZ == prevGyrZ &&
                          lastMagX == prevMagX && lastMagY == prevMagY && lastMagZ == prevMagZ;
    if (identicalFrame)
    {
        if (repeatedFrameCount < 255)
            repeatedFrameCount++;
    }
    else
    {
        repeatedFrameCount = 0;
    }
    prevAccX = lastAccX;
    prevAccY = lastAccY;
    prevAccZ = lastAccZ;
    prevGyrX = lastGyrX;
    prevGyrY = lastGyrY;
    prevGyrZ = lastGyrZ;
    prevMagX = lastMagX;
    prevMagY = lastMagY;
    prevMagZ = lastMagZ;
    havePreviousFrame = true;

    const double magAxes[3] = {lastMagX, lastMagY, lastMagZ};
    bool stuckRail = false;
    for (int axis = 0; axis < 3; axis++)
    {
        if (isNearAbs(magAxes[axis], ICM20948_AGMT_MAG_RAIL_UT, ICM20948_AGMT_MAG_RAIL_TOLERANCE_UT))
        {
            if (magRailCounts[axis] < 255)
                magRailCounts[axis]++;
        }
        else
        {
            magRailCounts[axis] = 0;
        }
        if (magRailCounts[axis] >= ICM20948_AGMT_MAG_RAIL_REJECT_SAMPLES)
            stuckRail = true;
    }
    lastMagValid =
#if defined(GWICM20948_USE_WOLLEWALD)
        wollewaldMagStatusValid && isfinite(magNorm) && magNorm > 1.0 && !stuckRail;
    snprintf(lastStatus, sizeof(lastStatus), "Wollewald read %s", lastMagValid ? "ok" : "mag invalid");
#else
        (imu.status == ICM_20948_Stat_Ok) && isfinite(magNorm) && magNorm > 1.0 && !stuckRail;

    if (imu.status != ICM_20948_Stat_Ok)
    {
        sensorErrors++;
        return accelPlausible && gyroPlausible;
    }
#endif
    if (repeatedFrameCount >= ICM20948_IDENTICAL_FRAME_REJECT_SAMPLES)
    {
        sensorErrors++;
        lastMagValid = false;
#if defined(GWICM20948_USE_WOLLEWALD)
        snprintf(lastStatus, sizeof(lastStatus), "Wollewald stale frame");
#endif
        return false;
    }
    return accelPlausible && gyroPlausible;
}

Vec3 GwIcm20948HardwareAdapter::readAccelG() const
{
    return Vec3(lastAccX, lastAccY, lastAccZ);
}

Vec3 GwIcm20948HardwareAdapter::readGyroDegPerSec() const
{
    return Vec3(lastGyrX, lastGyrY, lastGyrZ);
}

Vec3 GwIcm20948HardwareAdapter::readMagRaw() const
{
    return Vec3(lastMagX, lastMagY, lastMagZ);
}

bool GwIcm20948HardwareAdapter::readDmpQuaternion(Quaternion &out)
{
#if defined(GWICM20948_USE_WOLLEWALD)
    (void)out;
    dmpReadResult = DmpReadResult();
    return false;
#else
    dmpReadResult = DmpReadResult();
    haveDmpCompassSample = false;
    icm_20948_DMP_data_t data;
    ICM_20948_Status_e dmpStat;
    do
    {
        dmpStat = imu.readDMPdataFromFIFO(&data);
        if (dmpStat == ICM_20948_Stat_FIFONoDataAvail)
            dmpReadResult.fifoNoData = true;
        if (dmpStat != ICM_20948_Stat_Ok && dmpStat != ICM_20948_Stat_FIFOMoreDataAvail && dmpStat != ICM_20948_Stat_FIFONoDataAvail)
        {
            fifoErrors++;
            dmpReadResult.fifoError = true;
        }
        if (dmpStat == ICM_20948_Stat_Ok || dmpStat == ICM_20948_Stat_FIFOMoreDataAvail)
        {
            framesDrained++;
            dmpReadResult.framesRead++;
        }
        if ((dmpStat == ICM_20948_Stat_Ok || dmpStat == ICM_20948_Stat_FIFOMoreDataAvail) &&
            (data.header & DMP_header_bitmap_Quat9) > 0)
        {
            double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // 2^30
            double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
            double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
            double sumSq = q1 * q1 + q2 * q2 + q3 * q3;
            double q0 = sqrt(sumSq < 1.0 ? 1.0 - sumSq : 0.0);
            out.w = q0;
            out.x = q1;
            out.y = q2;
            out.z = q3;
            dmpReadResult.haveQuaternion = true;
        }
        // Same frame this cycle's Quat9 came from (or an earlier one drained
        // in this same do-while pass) can also carry the calibrated compass
        // field - captured here rather than via a second FIFO read, see
        // readDmpCompass()'s doc comment for why a second drain is wrong.
        if ((dmpStat == ICM_20948_Stat_Ok || dmpStat == ICM_20948_Stat_FIFOMoreDataAvail) &&
            (data.header & DMP_header_bitmap_Compass_Calibr) > 0)
        {
            lastDmpCompassRaw.x = ((double)data.Compass_Calibr.Data.X) / 65536.0; // 2^16, uT
            lastDmpCompassRaw.y = ((double)data.Compass_Calibr.Data.Y) / 65536.0;
            lastDmpCompassRaw.z = ((double)data.Compass_Calibr.Data.Z) / 65536.0;
            haveDmpCompassSample = true;
            dmpReadResult.haveCompass = true;
        }
        if (dmpReadResult.framesRead >= DMP_MAX_FIFO_FRAMES_PER_CYCLE && dmpStat == ICM_20948_Stat_FIFOMoreDataAvail)
        {
            fifoDrainLimits++;
            dmpReadResult.drainLimitHit = true;
            break;
        }
    } while (dmpStat == ICM_20948_Stat_FIFOMoreDataAvail);
    return dmpReadResult.haveQuaternion;
#endif
}

bool GwIcm20948HardwareAdapter::readDmpCompass(Vec3 &out)
{
    if (!haveDmpCompassSample)
        return false;
    out = lastDmpCompassRaw;
    return true;
}

bool GwIcm20948HardwareAdapter::resetFifo(GwLog *logger)
{
#if defined(GWICM20948_USE_WOLLEWALD)
    (void)logger;
    return true;
#else
    ICM_20948_Status_e stat = imu.resetFIFO();
    if (stat != ICM_20948_Stat_Ok)
    {
        sensorErrors++;
        if (logger)
            LOG_DEBUG(GwLog::ERROR, "ICM20948 resetFIFO failed: %s", imu.statusString());
        return false;
    }
    if (logger)
        LOG_DEBUG(GwLog::LOG, "ICM20948 FIFO reset ok");
    return true;
#endif
}
