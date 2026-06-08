#include <cstdint>
#pragma once

struct NavigationState {
    float distance;         // общая дистанция (м)
    float currentSpeed;     // текущая скорость (м/с)
    float avgSpeed;         // средняя скорость (м/с)
    float pace500;          // темп: секунды на 500м (как в гребле)

    uint32_t totalTime;     // общее время (мс)
    uint32_t strokeCount;   // количество гребков

    bool gpsValid;
    bool imuValid;
    bool gpsTrusted;
    bool valid;
};