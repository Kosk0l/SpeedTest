#include <memory>
#include "internal/imu/service/imurepo.h"
#include "internal/imu/domain/ImuData.h"

// Abstration - imu module, service
class imu {
private:
    std::shared_ptr<imurepo> driver; // DI
    ImuData lastValue; // кеш последнего значения
public:
    // Constructor
    imu(std::unique_ptr<imurepo> d) : driver(std::move(d)) {}

    // Получить данные из repository 
    ImuData read();

    // Обновить кеш
    void update();
    // Получить кеш
    ImuData get();
};

