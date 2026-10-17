#include "internal/gps/domain/GpsData.h"
#include "internal/gps/service/gps.h"
#include "internal/gps/domain/GpsRawData.h"

bool gps::update() {
    GpsRawData raw{};
    GpsData out{};

    // Вызов driver
    if (!driver || !driver->readRaw(raw)) {
        return false;
    }

    // Нормализация
    out.lat   = raw.lat;
    out.lon   = raw.lon;
    out.alt   = raw.alt;

    out.speed = raw.speed;
    out.velN  = raw.velN;
    out.velE  = raw.velE;
    out.sAcc  = raw.sAcc;
    out.hAcc  = raw.hAcc;
    out.pDop  = raw.pDop;

    out.fixType    = raw.fixType;
    out.fixOk      = raw.fixOk;
    out.satellites = raw.satellites;

    out.iTow     = raw.iTow;
    out.unixTime = raw.unixTime;
    out.time     = raw.time;

    // cache update
    lastValue = out;
    return true;
}

GpsData gps::get() const {
    return lastValue;
}

bool gps::init() {
    return driver->init();
}
