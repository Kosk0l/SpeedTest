#include <Arduino.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "internal/imu/service/imurepo.h"
#include "internal/imu/domain/ImuRawData.h"


class Icm20948Repo : public imurepo {
private:
    Adafruit_ICM20948 icm; // Обертка
    bool initialized = false;
public:
    bool init();
    bool readRaw(ImuRawData& out) override;
};

