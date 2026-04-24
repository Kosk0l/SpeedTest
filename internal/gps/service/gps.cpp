#include "GpsData.h"
#include "gps.h"
#include "GpsRawData.h"


GpsData gps::read() {
    GpsRawData raw;
    GpsData out{};
    
    // Вызов repository метода и записать в raw данные
    if (!driver->readRaw(raw) || !raw.valid) {
        out.valid = false;
        return out;
    }

    // Нормализация данных
    out.lat = raw.lat;
    out.lon = raw.lon;
    out.alt = raw.alt;

    out.time = raw.time;
    out.speed = raw.speed;

    out.valid = true;

    return out;
}


//====================================================================================================


void gps::update() {
    lastValue = read();
}


GpsData gps::get() {
    return lastValue;
}