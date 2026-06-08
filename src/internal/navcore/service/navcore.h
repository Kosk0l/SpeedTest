#pragma once

#include <cstdint>
#include "internal/gps/domain/GpsData.h"
#include "internal/imu/domain/ImuData.h"
#include "internal/navcore/domain/NavigationState.h"

class NavCore {
private:
    NavigationState state;

    // internal EKF state
    float x, y;
    float vx, vy;

    float P; // uncertainty (simplified covariance)

    // time
    uint32_t lastTime = 0;
    uint32_t startTime = 0;

    // GPS tracking
    bool gpsTrusted = false;
    uint32_t gpsBanUntil = 0;

    // helpers
    float lat0 = 0;
    float lon0 = 0;
    bool originSet = false;

private:
    float deg2rad(float d);
    void gpsToXY(double lat, double lon, float& x, float& y);
    float distance(float x1, float y1, float x2, float y2);

    bool detectSpoof(const GpsData& gps, float jump, float speed);

public:
    // Constructor
    NavCore();
    
    // main method - calculation 
    void update(const GpsData& gps, const ImuData& imu);
    // Return state
    const NavigationState& get() const;
};