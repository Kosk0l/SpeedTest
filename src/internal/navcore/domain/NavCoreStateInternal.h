#include <cstdint>
#pragma once

struct NavCoreStateInternal {
    float x = 0;   // meters
    float y = 0;

    float vx = 0;  // m/s
    float vy = 0;

    float p = 1.0f; // uncertainty (упрощённый covariance)
};