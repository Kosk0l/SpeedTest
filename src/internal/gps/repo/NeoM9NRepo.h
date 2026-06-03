#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "internal/gps/service/gpsrepo.h"

class NeoM9NRepo : public gpsrepo {
private:
    SFE_UBLOX_GNSS gnss;
    HardwareSerial& serial;
    bool initialized = false;

public:
    explicit NeoM9NRepo(HardwareSerial& s) : serial(s) {}

    bool init();
    bool readRaw(GpsRawData& out) override;
};