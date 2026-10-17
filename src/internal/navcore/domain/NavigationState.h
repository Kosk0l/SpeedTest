#include <cstdint>
#pragma once

struct NavigationState {
    float distance = 0.0f;      // пройденная дистанция по GPS (м)
    float currentSpeed = 0.0f;  // текущая скорость, сглаженная (м/с)
    float avgSpeed = 0.0f;      // средняя скорость за время движения (м/с)
    float pace500 = 0.0f;       // темп: секунд на 500 м, 0 — нет данных
    float strokeRate = 0.0f;    // темп гребли (гребков/мин), 0 — не гребём

    uint32_t totalTime = 0;     // общее время (мс)
    uint32_t movingTime = 0;    // время в движении (мс)
    uint32_t strokeCount = 0;   // количество гребков

    uint32_t utcTime = 0;       // UNIX-время с GPS (с), 0 — неизвестно

    uint8_t satellites = 0;     // спутников в решении
    float hAcc = 0.0f;          // горизонтальная точность GPS (м)

    bool gpsValid = false;      // пакеты приходят и есть фикс
    bool gpsTrusted = false;    // фикс достаточно точный, дистанция считается
    bool imuValid = false;
    bool valid = false;
    bool isMoving = false;      // лодка движется (по скорости GPS)
    bool inStroke = false;      // фаза проводки по IMU
};
