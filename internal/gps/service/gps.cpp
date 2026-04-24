#include "GpsData.h"
#include "gps.h"
#include "GpsRawData.h"


GpsData gps::read() {
    GpsRawData raw;
    GpsData out{};
    
    // Вызов repository метода; записать данные в raw
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
    GpsData data = read();

    if (data.valid) {
        lastValue = data;
    }
}


GpsData gps::get() {
    return lastValue;
}