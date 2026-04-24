#include <cstdint>


struct GpsRawData {
    double lat;
    double lon;
    double alt;

    float speed;
    uint32_t time;

    bool valid;
};