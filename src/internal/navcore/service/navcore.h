#pragma once

#include <cstdint>
#include "internal/gps/domain/GpsData.h"
#include "internal/imu/domain/ImuData.h"
#include "internal/navcore/domain/NavigationState.h"

class NavCore {
private:
    NavigationState state;

    // === Локальная система координат (ENU) ===
    float x = 0.0f, y = 0.0f;       // позиция (м)
    float vx = 0.0f, vy = 0.0f;     // скорость (м/с)

    // === Компенсация дрейфа IMU ===
    float axBias = 0.0f, ayBias = 0.0f;  // bias акселерометра

    // === GPS проекция ===
    double lat0 = 0.0, lon0 = 0.0;
    bool originSet = false;

    // === Тайминг ===
    uint32_t startTime = 0;
    uint32_t lastTime = 0;
    uint32_t lastGpsTime = 0;

    // === Адаптивное доверие к GPS (0.0 — полное недоверие, 1.0 — полное доверие) ===
    float gpsQuality = 0.0f;

    // === Детектор гребков ===
    bool inStroke = false;
    uint32_t lastStrokeTime = 0;

    // === Вспомогательные методы ===
    float deg2rad(float d);
    void gpsToXY(double lat, double lon, float& xOut, float& yOut);
    float calcDistance(float x1, float y1, float x2, float y2);
    float estimateGpsQuality(const GpsData& gps, float jump);
    void updateBias(const ImuData& imu, float dt);
    void detectStroke(const ImuData& imu, uint32_t now);

public:
    // Constructor
    NavCore();
    
    // main method - calculation 
    void update(const GpsData& gps, const ImuData& imu);
    // Return state
    const NavigationState& get() const;
};