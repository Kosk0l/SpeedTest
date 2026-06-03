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

    return gpsOk && imuOk && displayOk;

}

void App::update() {
    uint32_t now = millis();

    // GPS
    if (now - lastGpsUpdate >= 200) {
        gpsService.update();
        lastGpsUpdate = now;
    }

    // IMU
    if (now - lastImuUpdate >= 20) {
        imuService.update();
        lastImuUpdate = now;
    }

    navCore.update(
        gpsService.get(),
        imuService.get()
    );

    // Display
    if (now - lastDisplayUpdate >= 200) {
        displayService.update(navCore.get());
        lastDisplayUpdate = now;
    }

}