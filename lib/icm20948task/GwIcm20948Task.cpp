#include "GwIcm20948Task.h"
#include "GwHardware.h"
#include <N2kMessages.h>
#include <ICM_20948.h>

/*
  Deliberately NOT sending magnetic heading (PGN 127250) yet: the
  magnetometer needs a hard/soft-iron deviation calibration (compass swing)
  against this exact mounting position before its numbers mean anything -
  an uncalibrated heading on the N2K bus is worse than none, since something
  downstream (autopilot, plotter) could trust it. Roll/pitch from the
  accelerometer alone don't have that problem - they're geometrically valid
  as soon as the axis-to-boat mapping is right, which is why this first pass
  only sends PGN 127257 (Attitude).

  Roll/pitch sign convention below assumes the board's silkscreened X axis
  points toward the bow and Z points up when mounted flat - verify this
  against your actual mounting and adjust GWICM20948_AXIS_* if needed.
*/

#ifndef GWICM20948_SDA_PIN
#define GWICM20948_SDA_PIN -1
#endif
#ifndef GWICM20948_SCL_PIN
#define GWICM20948_SCL_PIN -1
#endif

static void runIcm20948Task(GwApi *api)
{
    GwLog *logger = api->getLogger();
    LOG_DEBUG(GwLog::LOG, "icm20948 task starting, sda=%d scl=%d", GWICM20948_SDA_PIN, GWICM20948_SCL_PIN);
    Wire.setTimeOut(1000); // bound any I2C hang instead of blocking this task forever
    bool wireOk = Wire.begin(GWICM20948_SDA_PIN, GWICM20948_SCL_PIN);
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

    ICM_20948_I2C imu;
    bool found = false;
    // AD0_VAL=1 -> 0x69 (default, ADR jumper open), AD0_VAL=0 -> 0x68 (jumper closed)
    for (int attempt = 0; attempt < 6 && !found; attempt++)
    {
        uint8_t ad0 = (attempt % 2 == 0) ? 1 : 0;
        LOG_DEBUG(GwLog::LOG, "ICM20948 init attempt %d (AD0=%d) starting", attempt, ad0);
        imu.begin(Wire, ad0);
        LOG_DEBUG(GwLog::LOG, "ICM20948 init attempt %d (AD0=%d) result: %s", attempt, ad0, imu.statusString());
        if (imu.status == ICM_20948_Stat_Ok)
        {
            found = true;
        }
        else
        {
            delay(300);
        }
    }
    if (!found)
    {
        LOG_DEBUG(GwLog::ERROR, "ICM20948 not found on sda=%d,scl=%d - task stopped", GWICM20948_SDA_PIN, GWICM20948_SCL_PIN);
        vTaskDelete(NULL);
        return;
    }
    LOG_DEBUG(GwLog::LOG, "ICM20948 found, starting attitude updates");
    unsigned char sid = 0;
    while (true)
    {
        delay(200);
        if (!imu.dataReady())
            continue;
        imu.getAGMT();
        double accX = imu.accX();
        double accY = imu.accY();
        double accZ = imu.accZ();
        double roll = atan2(accY, accZ);
        double pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));
        LOG_DEBUG(GwLog::DEBUG, "ICM20948 roll=%.3f pitch=%.3f (rad)", roll, pitch);
        tN2kMsg msg;
        SetN2kAttitude(msg, sid, N2kDoubleNA, pitch, roll);
        api->sendN2kMessage(msg);
        sid = (sid + 1) % 252;
    }
}

void initIcm20948(GwApi *api)
{
    GwLog *logger = api->getLogger();
    if (GWICM20948_SDA_PIN < 0 || GWICM20948_SCL_PIN < 0)
    {
        LOG_DEBUG(GwLog::LOG, "no ICM20948 pins defined for this board, task not started");
        return;
    }
    api->addUserTask(runIcm20948Task, String("icm20948Task"), 4000);
}
