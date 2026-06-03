#include "internal/imu/service/imu.h"

bool imu::update() {
    ImuRawData raw{};
    ImuData out{};

    // Вызов driver
    if (!driver->readRaw(raw) || !raw.valid) {
        return false;
    }

    // Валиданость raw
    if (!raw.valid) {
        return false;
    }

    // Нормализация
    out.ax = raw.ax;
    out.ay = raw.ay;
    out.az = raw.az;

    out.gx = raw.gx;
    out.gy = raw.gy;
    out.gz = raw.gz;

    out.time = raw.time;
    out.valid = true;

    // cache update
    lastValue = out;
    return true;
}

ImuData imu::get() {
    return lastValue;
}