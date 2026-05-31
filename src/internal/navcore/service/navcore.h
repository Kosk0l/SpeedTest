#include "internal/navcore/domain/NavigationState.h"
#include "internal/gps/domain/GpsData.h" // gps 
#include "internal/imu/domain/ImuData.h" // imu 


// Core module: Math + Memory
class NavCore {
private:
    NavigationState state{};

    GpsData prevGps{};
    bool hasPrev = false;

    uint32_t startTime = 0;
    float filteredSpeed = 0;
    uint32_t strokeCount = 0;

public:
    // Constructor
    NavCore();
    // main method - calculation 
    void update(const GpsData& gps, const ImuData& imu);
    // Return state
    const NavigationState& get() const;
};