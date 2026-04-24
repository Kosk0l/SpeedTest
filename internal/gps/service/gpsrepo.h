#include "GpsRawData.h"


// Intarface - Abstraction to DI with gps servive
class gpsrepo {
public:
    // Прочитать данные (true/false);
    // Данные Записываются в out по указателю
    virtual bool readRaw(GpsRawData& out) = 0;
    // Destructor
    virtual ~gpsrepo() = default;
};
