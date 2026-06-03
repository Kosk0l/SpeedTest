#include "internal/gps/domain/GpsRawData.h"
#pragma once

class gpsrepo {
public:
    virtual ~gpsrepo() = default;
    virtual bool init() = 0;

    // Получить сырые данные
    virtual bool readRaw(GpsRawData& out) = 0;
};