#include "internal/imu/repo/Icm20948Repo.h"

bool Icm20948Repo::init() {
    if (!icm.begin_I2C()) {
        initialized = false;
        return false;
    }

    initialized = true;
    return true;
}

bool Icm20948Repo::readRaw(ImuRawData& out) {
    if (!initialized) return false;

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;

    icm.getEvent(&accel, &gyro, &temp, &mag);

    // Акселерометр
    out.ax = accel.acceleration.x;
    out.ay = accel.acceleration.y;
    out.az = accel.acceleration.z;

    // Гироскоп
    out.gx = gyro.gyro.x;
    out.gy = gyro.gyro.y;
    out.gz = gyro.gyro.z;

    // Магнитометр
    out.mx = mag.magnetic.x;
    out.my = mag.magnetic.y;
    out.mz = mag.magnetic.z;

    // Время
    out.time = millis();

    return true;
}

// Serial.println("========== IMU ==========");
    // Serial.print("ACC: ");
    // Serial.print(out.ax);
    // Serial.print(" ");
    // Serial.print(out.ay);
    // Serial.print(" ");
    // Serial.println(out.az);
    // Serial.print("GYR: ");
    // Serial.print(out.gx);
    // Serial.print(" ");
    // Serial.print(out.gy);
    // Serial.print(" ");
    // Serial.println(out.gz);
    // Serial.print("MAG: ");
    // Serial.print(out.mx);
    // Serial.print(" ");
    // Serial.print(out.my);
    // Serial.print(" ");
    // Serial.println(out.mz);
    // Serial.println("");