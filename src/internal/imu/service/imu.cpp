#include "internal/imu/service/imu.h"


ImuData imu::read() {
    ImuRawData raw;
    ImuData out{};

    // Вызов repository метода; записать данные в raw
    if (!driver->readRaw(raw) || !raw.valid) {
        out.valid = false;
        return out;
    }

    out.ax = raw.ax;
    out.ay = raw.ay;
    out.az = raw.az;

    out.gx = raw.gx;
    out.gy = raw.gy;
    out.gz = raw.gz;

    out.time = raw.time;
    out.valid = true;

    return out;
}


//====================================================================================================


void imu::update() {
    ImuData data = read();

    if (data.valid) {
        lastValue = data;
    }
}


ImuData imu::get() {
    return lastValue;
}