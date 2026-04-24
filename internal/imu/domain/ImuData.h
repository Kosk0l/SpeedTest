#include <cstdint>

struct ImuData {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    uint32_t time;
    bool valid;
};