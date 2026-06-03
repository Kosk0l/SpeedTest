#include "app.h"

App::App()
    : gpsService(std::unique_ptr<gpsrepo>(new NeoM9NRepo(Serial1))),
      imuService(std::unique_ptr<imurepo>(new Icm20948Repo())),
      displayService(std::unique_ptr<displayrepo>(new SharpDisplayRepo()))
{}