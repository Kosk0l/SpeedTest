#include <memory>
#include "imurepo.h"

// Abstration - imu module, service
class imu {
private:
    std::shared_ptr<imurepo> imurepo; // DI
public:
    imu();
    ~imu();
};


// Constructor
imu::imu() {
}


// Destructor
imu::~imu() {
}
