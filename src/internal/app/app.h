#pragma once
#include <memory>

// services
#include "internal/gps/service/gps.h"
#include "internal/imu/service/imu.h"
#include "internal/navcore/service/navcore.h"
#include "internal/display/service/display.h"

// repos
#include "internal/gps/repo/NeoM9NRepo.h"
#include "internal/imu/repo/Icm20948Repo.h"
#include "internal/display/repo/SharpDisplay.h"

class App {
private:
    gps gpsService;
    imu imuService;
    NavCore navCore;
    display displayService;

public:
    App();

    void init();
    void update();
};