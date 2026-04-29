#include <internal/gps/repo/NeoM9NRepo.h>


bool NeoM9NRepo::init() {
    // UART
    serial.begin(38400, SERIAL_8N1, 16, 17);
    delay(100);

    // GNSS
    if (!gnss.begin(serial)) {
        initialized = false;
        return false;
    }

    // Частота обновления 
    gnss.setNavigationFrequency(5);
    // Проверить что получает PVT данные
    gnss.setAutoPVT(true);
    // уменьшить лишний вывод NMEA
    gnss.setI2COutput(COM_TYPE_UBX); 

    // 4. Проверка фикса
    int fixType = gnss.getFixType();

    initialized = true;
    return true;
}


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