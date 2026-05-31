#include <cstdint>
#pragma once

struct ImuRawData {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    uint32_t time;
    bool valid;
};