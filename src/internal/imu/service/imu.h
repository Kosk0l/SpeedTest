#include <memory>
#include "internal/imu/service/imurepo.h"
#include "internal/imu/domain/ImuData.h"

// Abstration - imu module, service
class imu {
private:
    std::unique_ptr<imurepo> driver; // DI
    ImuData lastValue; // кеш последнего значения
public:
    // Конструктор
    explicit imu(std::unique_ptr<imurepo> d) : driver(std::move(d)) {}

    // Обновить данные
    bool update();
    // Получить данные
    ImuData get();
};

