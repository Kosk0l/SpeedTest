#pragma once
#include <Arduino.h>
#include <memory>
#include "internal/gps/service/gpsrepo.h"
#include "internal/gps/domain/GpsData.h"

class gps {
private:
    std::unique_ptr<gpsrepo> driver; // DI
    GpsData lastValue{}; // Cache
public:
    // Конструктор
    explicit gps(std::unique_ptr<gpsrepo> d) : driver(std::move(d)) {}

    // Обновить данные 
    bool update();
    // Получение данных
    GpsData get() const;
};
