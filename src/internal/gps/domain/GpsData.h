#include <cstdint>
#pragma once

struct GpsData {
    double lat;
    double lon;
    double alt;

    float speed;          // м/с, доплеровская скорость над землёй (2D)
    float velN, velE;     // м/с, вектор скорости: север, восток
    float sAcc;           // м/с, оценка точности скорости
    float hAcc;           // м, оценка горизонтальной точности
    float pDop;

    uint8_t fixType;      // 0 — нет, 2 — 2D, 3 — 3D, 4 — GNSS+DR
    bool fixOk;           // приёмник считает решение валидным
    uint8_t satellites;   // спутников в решении

    uint32_t iTow;        // мс, время недели GPS — метка навигационной эпохи
    uint32_t unixTime;    // с, 0 — время ещё не известно

    uint32_t time;        // millis() получения пакета, 0 — пакетов ещё не было
};
