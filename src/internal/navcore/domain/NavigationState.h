#include <cstdint>
#pragma once

struct NavigationState {
    float distance = 0.0f;      // общая дистанция (м)
    float currentSpeed = 0.0f;  // текущая скорость (м/с)
    float avgSpeed = 0.0f;      // средняя скорость (м/с)
    float pace500 = 0.0f;       // темп: секунды на 500м (как в гребле)

    uint32_t totalTime = 0;     // общее время (мс)
    uint32_t strokeCount = 0;   // количество гребков

    bool valid = false;
};