#include "internal/navcore/service/navcore.h"
#include <Arduino.h>
#include <math.h>

NavCore::NavCore()
    : prevGps{},
    hasPrev(false),
    startTime(0),
    filteredSpeed(0.0f),
    strokeCount(0)
{
    state = {};
}

// Вычисление дистанции
float distanceMeters(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0; // радиус Земли (м)

    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;

    lat1 *= DEG_TO_RAD;
    lat2 *= DEG_TO_RAD;

    double a = sin(dLat / 2) * sin(dLat / 2) +
               sin(dLon / 2) * sin(dLon / 2) *
               cos(lat1) * cos(lat2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c;
}

// Основной метод обработки 
void NavCore::update(const GpsData& gps, const ImuData& imu) {
    if (!gps.valid) {
        state.valid = false;
        return;
    }

    uint32_t now = gps.time;

    // Start
    if (!hasPrev) {
        prevGps = gps;
        hasPrev = true;
        startTime = now;

        state.valid = true;
        return;
    }

    // Time total
    state.totalTime = now - startTime;

    // Distation
    float dist = distanceMeters(prevGps.lat, prevGps.lon, gps.lat, gps.lon);
    if (dist > 1.0f && dist < 50.0f) { // защита от скачков 
        state.distance += dist;
    }

    // Speed 
    float rawSpeed = gps.speed;
    filteredSpeed = 0.2f * rawSpeed + 0.8f * filteredSpeed;
    state.currentSpeed = filteredSpeed;

    // avgRace
    if (state.totalTime > 0) {
        state.avgSpeed = state.distance / (state.totalTime / 1000.0f);
    }

    // pace 500
    if (filteredSpeed > 0.1f) {
        state.pace500 = 500.0f / filteredSpeed;
    }

    // Strocks
    static bool strokeActive = false;

    if (imu.az > 12.0f && !strokeActive) {
        strokeCount++;
        strokeActive = true;
    }

    if (imu.az < 9.0f) {
        strokeActive = false;
    }

    state.strokeCount = strokeCount;

    state.valid = true;
    prevGps = gps;
}

// Получить данные
const NavigationState& NavCore::get() const {
    return state;
}
