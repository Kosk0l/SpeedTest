#include <memory>
#include <Arduino.h>
#include "internal/gps/service/gpsrepo.h"
#include "internal/gps/domain/GpsData.h"



class gps {
private:
    std::unique_ptr<gpsrepo> driver; // DI
    GpsData lastValue; // кеш последнего значения
public:
    // Constructor
    gps(std::unique_ptr<gpsrepo> d) : driver(std::move(d)){}
    
    // Получить данные из repository
    GpsData read();

    // Обновить кеш
    void update();
    // Получить кеш
    GpsData get();
};

