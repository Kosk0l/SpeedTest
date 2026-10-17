#include "app.h"
#include "internal/display/service/display.h"

App::App() 
	: gpsService(std::unique_ptr<gpsrepo>(new NeoM9NRepo(Serial1))),
    imuService(std::unique_ptr<imurepo>(new Icm20948Repo())),
    displayService(std::unique_ptr<displayrepo>(new SharpDisplayRepo()))
{}

bool App::init() {
    bool gpsOk = gpsService.init();
    bool imuOk = imuService.init();
    bool displayOk = displayService.init();

    Serial.printf("init: gps=%d imu=%d display=%d\n", gpsOk, imuOk, displayOk);

    return gpsOk && imuOk && displayOk;
}

void App::update() {
    uint32_t now = millis();

    // GPS: опрашиваем на каждом цикле. В режиме autoPVT getPVT() не блокирует,
    // а редкий опрос переполнял буфер UART и терял пакеты
    gpsService.update();

    // IMU
    if (now - lastImuUpdate >= 20) {
        imuService.update();
        lastImuUpdate = now;
    }

    // Core
    navCore.update(gpsService.get(), imuService.get());

    // Display
    if (now - lastDisplayUpdate >= 500) {
        displayService.update(navCore.get());
        lastDisplayUpdate = now;
    }

    // Диагностика в Serial
    if (now - lastDebugPrint >= 1000) {
        GpsData gps = gpsService.get();
        const NavigationState& state = navCore.get();
        long gpsAge = gps.time ? (long)(millis() - gps.time) : -1;  // -1 — пакетов не было

        Serial.printf("gps: age=%ldms fix=%u ok=%d sat=%u hAcc=%.1fm sAcc=%.2fm/s unix=%lu"
                      " | trusted=%d moving=%d dist=%.1fm strokes=%lu\n",
                      gpsAge, gps.fixType, gps.fixOk, gps.satellites, gps.hAcc, gps.sAcc,
                      (unsigned long)gps.unixTime,
                      state.gpsTrusted, state.isMoving, state.distance, (unsigned long)state.strokeCount);
        lastDebugPrint = now;
    }
}