#include "GwIcm20948CalControlTask.h"
#include "GwJsonDocument.h"
#include "GwSynchronized.h"

void Icm20948CalControl::begin(GwApi *apiIn, GwConfigHandler *configIn)
{
    api = apiIn;
    config = configIn;
    lock = xSemaphoreCreateMutex();

    api->registerRequestHandler("calControl", [this](AsyncWebServerRequest *request)
                                 { handleCalControl(request); });
    api->registerRequestHandler("gyroCalControl", [this](AsyncWebServerRequest *request)
                                 { handleGyroCalControl(request); });
    api->registerRequestHandler("gyroCalStatus", [this](AsyncWebServerRequest *request)
                                 { handleGyroCalStatus(request); });
    api->registerRequestHandler("magCalControl", [this](AsyncWebServerRequest *request)
                                 { handleMagCalControl(request); });
    api->registerRequestHandler("magCalStatus", [this](AsyncWebServerRequest *request)
                                 { handleMagCalStatus(request); });
}

void Icm20948CalControl::feedGyroSample(const Vec3 &gyroBoatDegPerSec, double accelMagG)
{
    GWSYNCHRONIZED(lock);
    gyroCal.addSample(gyroBoatDegPerSec, accelMagG);
}

void Icm20948CalControl::feedMagSample(double magBoatX, double magBoatY)
{
    GWSYNCHRONIZED(lock);
    magCal2D.addSample(magBoatX, magBoatY);
}

void Icm20948CalControl::readCurrentCalibration(ImuCalibration &outCal, DeviationTable &outDeviationTable, bool &outDeviationEnabled)
{
    String calJsonStr;
    config->getValue(calJsonStr, GwConfigDefinitions::icmCalJson, "");
    if (calJsonStr.length() > 0)
    {
        MountOrientation ignoredOrientation; // icmOrientation config remains authoritative - see class comment
        std::string err;
        if (ImuCalibrationJson::importJson(calJsonStr.c_str(), outCal, ignoredOrientation, outDeviationTable, outDeviationEnabled, err))
            return;
        // Stored value is present but doesn't validate - shouldn't happen
        // (this class only ever writes what its own exportJson produced),
        // but fail safely toward the legacy fallback below rather than
        // propagating a bad cached blob.
    }

    float magXOffset = 0, magYOffset = 0, hdgOffsetDeg = 0;
    config->getValue(magXOffset, GwConfigDefinitions::icmMagXOff, 0.0f);
    config->getValue(magYOffset, GwConfigDefinitions::icmMagYOff, 0.0f);
    config->getValue(hdgOffsetDeg, GwConfigDefinitions::icmHdgOff, 0.0f);
    outCal = ImuCalibrationOps::migrateFromLegacy(magXOffset, magYOffset, hdgOffsetDeg);
    outDeviationTable = DeviationTable();
    outDeviationEnabled = false;
}

void Icm20948CalControl::persistCalibration(const ImuCalibration &cal, const DeviationTable &deviationTable, bool deviationEnabled)
{
    int orientationIdx = 0;
    config->getValue(orientationIdx, GwConfigDefinitions::icmOrientation, 0);
    std::string json = ImuCalibrationJson::exportJson(cal, static_cast<MountOrientation>(orientationIdx), deviationTable, deviationEnabled);
    String jsonStr(json.c_str());
    config->setValue(GwConfigDefinitions::icmCalJson, jsonStr);
    config->updateValue(GwConfigDefinitions::icmCalJson, jsonStr);
    String devStr = deviationEnabled ? "true" : "false";
    config->setValue(GwConfigDefinitions::icmDevEnable, devStr);
    config->updateValue(GwConfigDefinitions::icmDevEnable, devStr);
}

void Icm20948CalControl::handleCalControl(AsyncWebServerRequest *request)
{
    String action = request->hasParam("action") ? request->getParam("action")->value() : "";

    if (action == "export")
    {
        ImuCalibration cal;
        DeviationTable devTable;
        bool devEnabled = false;
        readCurrentCalibration(cal, devTable, devEnabled);
        int orientationIdx = 0;
        config->getValue(orientationIdx, GwConfigDefinitions::icmOrientation, 0);
        std::string json = ImuCalibrationJson::exportJson(cal, static_cast<MountOrientation>(orientationIdx), devTable, devEnabled);
        request->send(200, "application/json", String(json.c_str()));
        return;
    }

    if (action == "reset")
    {
        persistCalibration(ImuCalibrationOps::identityDefault(), DeviationTable(), false);
        request->send(200, "application/json", "{\"status\":\"OK\"}");
        return;
    }

    if (action == "import")
    {
        if (!request->hasParam("json"))
        {
            request->send(200, "application/json", "{\"status\":\"error\",\"message\":\"missing json parameter\"}");
            return;
        }
        String jsonParam = request->getParam("json")->value();
        ImuCalibration outCal;
        MountOrientation outOrientation;
        DeviationTable outDevTable;
        bool outDevEnabled;
        std::string err;
        bool ok = ImuCalibrationJson::importJson(std::string(jsonParam.c_str()), outCal, outOrientation, outDevTable, outDevEnabled, err);
        if (!ok)
        {
            GwJsonDocument doc(384);
            doc["status"] = "error";
            doc["message"] = err.c_str();
            String out;
            serializeJson(doc, out);
            request->send(200, "application/json", out);
            return;
        }
        String orientationStr(static_cast<int>(outOrientation));
        config->setValue(GwConfigDefinitions::icmOrientation, orientationStr);
        config->updateValue(GwConfigDefinitions::icmOrientation, orientationStr);
        persistCalibration(outCal, outDevTable, outDevEnabled);
        request->send(200, "application/json", "{\"status\":\"OK\"}");
        return;
    }

    request->send(200, "application/json", "{\"status\":\"error\",\"message\":\"unknown action\"}");
}

void Icm20948CalControl::handleGyroCalControl(AsyncWebServerRequest *request)
{
    String action = request->hasParam("action") ? request->getParam("action")->value() : "";
    GWSYNCHRONIZED(lock);

    if (action == "start")
    {
        gyroCal.start();
        request->send(200, "application/json", "{\"status\":\"OK\"}");
        return;
    }
    if (action == "cancel")
    {
        gyroCal.cancel();
        request->send(200, "application/json", "{\"status\":\"OK\"}");
        return;
    }
    if (action == "save")
    {
        if (gyroCal.state() != GyroCalState::Done)
        {
            request->send(200, "application/json", "{\"status\":\"error\",\"message\":\"no completed gyro calibration to save\"}");
            return;
        }
        Vec3 bias = gyroCal.resultBiasDegPerSec();
        ImuCalibration cal;
        DeviationTable devTable;
        bool devEnabled = false;
        readCurrentCalibration(cal, devTable, devEnabled);
        cal.gyroBias[0] = bias.x;
        cal.gyroBias[1] = bias.y;
        cal.gyroBias[2] = bias.z;
        cal.gyroCalibrationValid = true;
        cal.calibrationSequence++;
        persistCalibration(cal, devTable, devEnabled);
        gyroCal.cancel();
        request->send(200, "application/json", "{\"status\":\"OK\"}");
        return;
    }
    request->send(200, "application/json", "{\"status\":\"error\",\"message\":\"unknown action\"}");
}

void Icm20948CalControl::handleGyroCalStatus(AsyncWebServerRequest *request)
{
    GwJsonDocument doc(320);
    {
        GWSYNCHRONIZED(lock);
        GyroCalState st = gyroCal.state();
        doc["state"] = (st == GyroCalState::Idle) ? "idle" : (st == GyroCalState::Collecting) ? "collecting"
                                                          : (st == GyroCalState::Done)          ? "done"
                                                                                                 : "failed";
        doc["sampleCount"] = gyroCal.sampleCount();
        doc["requiredSamples"] = GyroCalConfig().requiredSamples;
        if (st == GyroCalState::Done)
        {
            Vec3 bias = gyroCal.resultBiasDegPerSec();
            Vec3 stddev = gyroCal.resultStdDevDegPerSec();
            doc["biasX"] = bias.x;
            doc["biasY"] = bias.y;
            doc["biasZ"] = bias.z;
            doc["stdDevX"] = stddev.x;
            doc["stdDevY"] = stddev.y;
            doc["stdDevZ"] = stddev.z;
        }
    }
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
}

void Icm20948CalControl::handleMagCalControl(AsyncWebServerRequest *request)
{
    String action = request->hasParam("action") ? request->getParam("action")->value() : "";
    GWSYNCHRONIZED(lock);

    if (action == "start")
    {
        magCal2D.start();
        request->send(200, "application/json", "{\"status\":\"OK\"}");
        return;
    }
    if (action == "stop")
    {
        magCal2D.stop();
        request->send(200, "application/json", "{\"status\":\"OK\"}");
        return;
    }
    if (action == "cancel")
    {
        magCal2D.cancel();
        request->send(200, "application/json", "{\"status\":\"OK\"}");
        return;
    }
    if (action == "save")
    {
        if (magCal2D.state() != MagCal2DState::Ready)
        {
            request->send(200, "application/json", "{\"status\":\"error\",\"message\":\"no completed magnetometer calibration to save\"}");
            return;
        }
        ImuCalibration cal;
        DeviationTable devTable;
        bool devEnabled = false;
        readCurrentCalibration(cal, devTable, devEnabled);
        cal.magBias[0] = magCal2D.resultBiasX();
        cal.magBias[1] = magCal2D.resultBiasY();
        // Z bias/matrix intentionally left untouched - a boat swing never
        // covers the Z axis, see ImuMagCal2D's class comment.
        cal.magMatrix[0][0] = magCal2D.resultScaleX();
        cal.magMatrix[1][1] = magCal2D.resultScaleY();
        cal.magCalibrationValid = true;
        cal.magCalibrationQuality = magCal2D.resultQuality();
        cal.calibrationSequence++;
        persistCalibration(cal, devTable, devEnabled);
        magCal2D.cancel();
        request->send(200, "application/json", "{\"status\":\"OK\"}");
        return;
    }
    request->send(200, "application/json", "{\"status\":\"error\",\"message\":\"unknown action\"}");
}

void Icm20948CalControl::handleMagCalStatus(AsyncWebServerRequest *request)
{
    GwJsonDocument doc(384);
    {
        GWSYNCHRONIZED(lock);
        MagCal2DState st = magCal2D.state();
        doc["state"] = (st == MagCal2DState::Idle) ? "idle" : (st == MagCal2DState::Collecting) ? "collecting"
                                                            : (st == MagCal2DState::Ready)        ? "ready"
                                                                                                    : "failed";
        doc["sampleCount"] = magCal2D.sampleCount();
        doc["coverageFraction"] = magCal2D.coverageFraction();
        doc["spanDeg"] = magCal2D.spanDeg();
        if (st == MagCal2DState::Ready)
        {
            doc["biasX"] = magCal2D.resultBiasX();
            doc["biasY"] = magCal2D.resultBiasY();
            doc["scaleX"] = magCal2D.resultScaleX();
            doc["scaleY"] = magCal2D.resultScaleY();
            doc["quality"] = magCal2D.resultQuality();
        }
        else if (st == MagCal2DState::Failed)
        {
            doc["failureReason"] = magCal2D.failureReason();
        }
    }
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
}
