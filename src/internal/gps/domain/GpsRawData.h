#include <cstdint>
#pragma once

struct GpsRawData {
    double lat;
    double lon;
    double alt;

    float speed;
    uint32_t time;
};