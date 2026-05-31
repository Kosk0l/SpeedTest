#include <cstdint>
#pragma once

struct GpsData {
    double lat;
    double lon;
    double alt;

    float speed; // м/с
    uint32_t time;

    bool valid;
};