#include "app.h"
#include "internal/display/service/display.h"

App::App() 
	: gpsService(std::unique_ptr<gpsrepo>(new NeoM9NRepo(Serial1))),
    imuService(std::unique_ptr<imurepo>(new Icm20948Repo())),
    displayService(std::unique_ptr<displayrepo>(new SharpDisplayRepo()))
{}

bool App::init() {
	
    return true;
}

void App::update() {
    // обновление датчиков
    gpsService.update();
    imuService.update();

    // получение доменных данных
    GpsData gpsData = gpsService.get();
    ImuData imuData = imuService.get();

    // вычисления
    navCore.update(gpsData, imuData);

    // итоговое состояние
    const NavigationState& state = navCore.get();

    // вывод
    displayService.update(state);
}