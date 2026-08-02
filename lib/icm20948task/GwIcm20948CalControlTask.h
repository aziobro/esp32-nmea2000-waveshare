#pragma once
#include "GwApi.h"
#include "GWConfig.h"
#include "ImuTypes.h"
#include "ImuCalibration.h"
#include "ImuCalibrationJson.h"
#include "ImuDeviationTable.h"
#include "ImuGyroCal.h"
#include "ImuMagCal2D.h"
#include <freertos/semphr.h>

/*
  Owns the two interactive, HTTP-controlled calibration engines
  (stationary gyro bias via GyroCalEngine, live "boat swing" magnetometer
  fit via ImuMagCal2D) plus the calibration JSON export/import/reset
  endpoints. Registered once from runIcm20948Task, same lifetime/ownership
  pattern as Icm20948Capture (Phase 2) and Icm20948WebData.

  Persistence model: the full advanced calibration (magnetometer
  bias/matrix, gyro bias, fixed heading offset, deviation table) is stored
  as ONE JSON blob in the icmCalJson config field (schema: see
  ImuCalibrationJson.h), rather than one NVS key per number - this project
  enforces a 15-character NVS key name limit (see extra_script.py) that
  makes a 9-cell matrix + everything else impractical as individual keys,
  and reusing the exact schema Phase 3 already built/validated means
  "what gets persisted" and "what importJson() accepts" are provably the
  same thing. MountOrientation (icmOrientation) and the deviation-table
  enable flag (icmDevEnable) remain their own existing config fields -
  only Import writes those (Reset and the gyro/mag "save" actions leave
  mounting orientation alone, and leave the deviation table/enable flag
  untouched unless the import payload explicitly carries a table).

  Every "save"-style HTTP action here calls GwConfigHandler::setValue()
  (immediate in-memory update, so the very next task loop cycle picks it
  up - same "re-read config every cycle, no reboot needed" pattern this
  file already uses for every other icmXXX field) followed by
  updateValue() (synchronous NVS write, so it survives a power cycle) -
  deliberately NOT the full config-save flow in src/main.cpp, which
  restarts the device; an interactive calibration workflow shouldn't
  require a reboot to see its own result. This does mean a config value
  can change on another thread while the main loop's own config->getValue
  call reads it (GwConfigHandler has no internal lock) - the same
  characteristic already exists project-wide for every config read/write
  pair since nothing else previously wrote a config value without
  restarting immediately after; a project-wide fix is out of scope here.
*/
class Icm20948CalControl
{
public:
    void begin(GwApi *api, GwConfigHandler *config);

    // Called every IMU loop cycle - cheap, internally no-ops unless the
    // corresponding engine is actively Collecting.
    void feedGyroSample(const Vec3 &gyroBoatDegPerSec, double accelMagG);
    void feedMagSample(double magBoatX, double magBoatY);

private:
    GwApi *api = nullptr;
    GwConfigHandler *config = nullptr;
    SemaphoreHandle_t lock = nullptr;

    GyroCalEngine gyroCal;
    ImuMagCal2D magCal2D;

    // Reads icmCalJson (falling back to the legacy hard-iron fields if
    // empty/invalid) into outCal/outDeviationTable/outDeviationEnabled.
    // MountOrientation is intentionally not returned here - callers that
    // need it read icmOrientation directly, which remains authoritative.
    void readCurrentCalibration(ImuCalibration &outCal, DeviationTable &outDeviationTable, bool &outDeviationEnabled);

    // Serializes cal/deviationTable/deviationEnabled (with the CURRENT
    // icmOrientation) and persists to icmCalJson (setValue + updateValue,
    // see class comment).
    void persistCalibration(const ImuCalibration &cal, const DeviationTable &deviationTable, bool deviationEnabled);

    void handleCalControl(AsyncWebServerRequest *request);
    void handleGyroCalControl(AsyncWebServerRequest *request);
    void handleGyroCalStatus(AsyncWebServerRequest *request);
    void handleMagCalControl(AsyncWebServerRequest *request);
    void handleMagCalStatus(AsyncWebServerRequest *request);
};
