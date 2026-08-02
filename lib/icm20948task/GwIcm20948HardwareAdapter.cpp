#include "GwIcm20948HardwareAdapter.h"
#include <Wire.h>

bool GwIcm20948HardwareAdapter::begin(GwLog *logger, int sdaPin, int sclPin)
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
    delay(2500);
    Wire.setTimeOut(1000); // bound any I2C hang instead of blocking this task forever
    bool wireOk = Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000);
    LOG_DEBUG(GwLog::LOG, "Wire.begin returned %d", (int)wireOk);

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

    bool found = false;
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
    if (!found)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 not found on sda=%d,scl=%d", sdaPin, sclPin);
        return false;
    }
    LOG_DEBUG(GwLog::LOG, "ICM20948 found");
    return true;
}

void GwIcm20948HardwareAdapter::setFullScale(GwLog *logger, int accelRangeIdx, int gyrRangeIdx)
{
    ICM_20948_fss_t fss;
    fss.a = (uint8_t)accelRangeIdx;
    fss.g = (uint8_t)gyrRangeIdx;
    imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
    LOG_DEBUG(GwLog::LOG, "ICM20948 setFullScale acc=%d gyr=%d result: %s", accelRangeIdx, gyrRangeIdx, imu.statusString());
}

bool GwIcm20948HardwareAdapter::initDmp(GwLog *logger)
{
    bool ok = true;
    ok &= (imu.initializeDMP() == ICM_20948_Stat_Ok);
    ok &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    ok &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // 0 = maximum rate
    ok &= (imu.enableFIFO() == ICM_20948_Stat_Ok);
    ok &= (imu.enableDMP() == ICM_20948_Stat_Ok);
    ok &= (imu.resetDMP() == ICM_20948_Stat_Ok);
    ok &= (imu.resetFIFO() == ICM_20948_Stat_Ok);
    LOG_DEBUG(ok ? GwLog::LOG : GwLog::ERROR, "ICM20948 DMP init %s (last status: %s)",
              ok ? "ok" : "FAILED, falling back to plain accel/mag math", imu.statusString());
    return ok;
}

bool GwIcm20948HardwareAdapter::dataReady()
{
    return imu.dataReady();
}

void GwIcm20948HardwareAdapter::readAGMT()
{
    imu.getAGMT();
    // accX/Y/Z from the library are in mg (milli-g) - store in g.
    lastAccX = imu.accX() / 1000.0;
    lastAccY = imu.accY() / 1000.0;
    lastAccZ = imu.accZ() / 1000.0;
    lastGyrX = imu.gyrX();
    lastGyrY = imu.gyrY();
    lastGyrZ = imu.gyrZ();
    lastMagX = imu.magX();
    lastMagY = imu.magY();
    lastMagZ = imu.magZ();
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
    bool haveSample = false;
    icm_20948_DMP_data_t data;
    ICM_20948_Status_e dmpStat;
    do
    {
        dmpStat = imu.readDMPdataFromFIFO(&data);
        if (dmpStat != ICM_20948_Stat_Ok && dmpStat != ICM_20948_Stat_FIFOMoreDataAvail && dmpStat != ICM_20948_Stat_FIFONoDataAvail)
            fifoErrors++;
        if (dmpStat == ICM_20948_Stat_Ok || dmpStat == ICM_20948_Stat_FIFOMoreDataAvail)
            framesDrained++;
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
            haveSample = true;
        }
    } while (dmpStat == ICM_20948_Stat_FIFOMoreDataAvail);
    return haveSample;
}
