#include <internal/gps/repo/NeoM9NRepo.h>

bool NeoM9NRepo::init() {
    // UART. Буфер увеличиваем до begin(): стандартных 256 байт мало
    serial.setRxBufferSize(1024);
    serial.begin(38400, SERIAL_8N1, 16, 17);
    delay(100);

    // GNSS
    if (!gnss.begin(serial)) {
        initialized = false;
        return false;
    }

    // Только UBX на UART: NMEA забивает буфер, и PVT-пакеты теряются
    gnss.setUART1Output(COM_TYPE_UBX);
    // Частота обновления
    gnss.setNavigationFrequency(5);
    // Модель динамики: без допущений о высоте (озёра и реки не на уровне моря)
    gnss.setDynamicModel(DYN_MODEL_PORTABLE);
    // PVT приходит сам на каждой эпохе, getPVT() не блокирует
    gnss.setAutoPVT(true);

    initialized = true;
    return true;
}


bool NeoM9NRepo::readRaw(GpsRawData& out) {
    if (!initialized) return false;

    // Проверка, есть ли новые данные
    if (!gnss.getPVT()) {
        return false;
    }

    // Собрать данные с устройства
    out.iTow       = gnss.getTimeOfWeek();
    out.fixType    = gnss.getFixType();
    out.fixOk      = gnss.getGnssFixOk() && !gnss.getInvalidLlh();
    out.satellites = gnss.getSIV();

    long lat = gnss.getLatitude();
    long lon = gnss.getLongitude();
    long alt = gnss.getAltitude();

    // Нормализация 1
    out.lat = lat / 1e7;
    out.lon = lon / 1e7;
    out.alt = alt / 1000.0;

    out.speed = gnss.getGroundSpeed() / 1000.0f;       // мм/с -> м/с
    out.velN  = gnss.getNedNorthVel() / 1000.0f;
    out.velE  = gnss.getNedEastVel() / 1000.0f;
    out.sAcc  = gnss.getSpeedAccEst() / 1000.0f;       // мм/с -> м/с
    // В PVT hAcc беззнаковый, библиотека отдаёт int32: без фикса 0xFFFFFFFF стал бы −1 мм
    out.hAcc  = (uint32_t)gnss.getHorizontalAccEst() / 1000.0f;  // мм -> м
    out.pDop  = gnss.getPDOP() / 100.0f;

    // getUnixEpoch() берёт дни из таблицы на 2020–2099 годы без проверки границ
    uint16_t year = gnss.getYear();
    bool timeValid = gnss.getDateValid() && gnss.getTimeValid() && year >= 2020 && year <= 2099;
    out.unixTime = timeValid ? gnss.getUnixEpoch() : 0;

    out.time = millis();

    return true;
}

// Serial.println("========== GPS ==========");
    // Serial.print("LAT: ");
    // Serial.println(out.lat, 7);
    // Serial.print("LON: ");
    // Serial.println(out.lon, 7);
    // Serial.print("ALT: ");
    // Serial.print(out.alt);
    // Serial.println(" m");
    // Serial.print("SPD: ");
    // Serial.print(out.speed);
    // Serial.println(" m/s");
    // Serial.print("SAT: ");
    // Serial.println(gnss.getSIV());
    // Serial.println("");
