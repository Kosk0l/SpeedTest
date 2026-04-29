#include <cstdint>


struct ImuData {
    float ax, ay, az; // Ускорение (Акселерометр)
    float gx, gy, gz; // Угловая скорость (Гироскоп)
    float mx, my, mz; // Магнитное поле

    uint32_t time;
    bool valid;
};