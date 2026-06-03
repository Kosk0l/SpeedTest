#pragma once
#include <cstdint>

struct DisplayData {
    float distance;
    float currentSpeed;
    float avgSpeed;
    float pace500;

    uint32_t totalTime;
    uint32_t strokeCount;

    bool valid;
};