#include "ImuRawData.h"


// Interface - Abstraction to DI with imu service
class imurepo {
public:
    virtual bool readRaw(ImuRawData& out) = 0;
};