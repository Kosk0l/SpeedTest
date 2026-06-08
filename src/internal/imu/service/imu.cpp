#include "internal/imu/service/imu.h"

bool imu::update() {
    ImuRawData raw{};
    ImuData out{};

    // Вызов driver
    if (!driver->readRaw(raw)) {
        return false;
    }

    // Нормализация
    out.ax = raw.ax;
    out.ay = raw.ay;
    out.az = raw.az;

    out.gx = raw.gx;
    out.gy = raw.gy;
    out.gz = raw.gz;

    out.mx = raw.mx;
    out.my = raw.my;
    out.mz = raw.mz;

    out.time = raw.time;

    // cache update
    lastValue = out;
    return true;
}

ImuData imu::get() {
    return lastValue;
}

bool imu::init() {
    return driver->init();
}