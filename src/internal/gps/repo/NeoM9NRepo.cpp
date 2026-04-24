#include <internal/gps/repo/NeoM9NRepo.h>

bool NeoM9NRepo::readRaw(GpsRawData& out) {
    if (!initialized) return false;

    // Проверка, есть ли новые данные
    if (!gnss.getPVT()) {
        out.valid = false;
        return true; 
    }

    // Собрать данные с устройства
    long lat = gnss.getLatitude();
    long lon = gnss.getLongitude(); 
    long alt = gnss.getAltitude();

    // Валидация
    if (lat == 0 && lon == 0) {
        out.valid = false;
        return true;
    }

    // Нормализация 1
    out.lat = lat / 1e7;
    out.lon = lon / 1e7;
    out.alt = alt / 1000.0; 

    out.time = millis();
    out.speed = gnss.getGroundSpeed() / 1000.0;

    out.valid = true;

    return true;
}