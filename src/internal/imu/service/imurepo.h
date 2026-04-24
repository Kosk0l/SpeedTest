#include "internal/imu/domain/ImuRawData.h"


// Interface - Abstraction to DI with imu service
class imurepo {
public:
    // Прочитать данные (true/false);
    // Данные Записываются в out по указателю
    virtual bool readRaw(ImuRawData& out) = 0;
    // Destructor
    virtual ~imurepo() = default;
};