#include "internal/navcore/service/navcore.h"
#include <Arduino.h>
#include <math.h>

NavCore::NavCore() {
    x = y = 0;
    vx = vy = 0;
    P = 1.0f;
    state = {};
}

float NavCore::deg2rad(float d) {
    return d * 0.01745329251f;
}

// very rough local projection (good enough for small distances)
void NavCore::gpsToXY(double lat, double lon, float& xOut, float& yOut) {
    if (!originSet) {
        lat0 = lat;
        lon0 = lon;
        originSet = true;
    }

    const float R = 6371000.0f;

    float dLat = deg2rad(lat - lat0);
    float dLon = deg2rad(lon - lon0);

    xOut = R * dLon * cos(deg2rad(lat0));
    yOut = R * dLat;
}

float NavCore::distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx*dx + dy*dy);
}

bool NavCore::detectSpoof(const GpsData& gps, float jump, float speed) {
    if (jump > 60.0f) return true;
    if (speed > 12.0f) return true;
    if (jump > 25.0f && speed < 0.2f) return true;
    return false;
}

void NavCore::update(const GpsData& gps, const ImuData& imu) {
    uint32_t now = millis();

    if (startTime == 0) {
        startTime = now;
        lastTime = now;
    }

    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0) dt = 0.01f;
    lastTime = now;

    state.totalTime = now - startTime;

    // IMU PREDICTION STEP
    vx += imu.ax * dt;
    vy += imu.ay * dt;

    x += vx * dt;
    y += vy * dt;

    // damping (friction model)
    vx *= 0.98f;
    vy *= 0.98f;

    // GPS MEASUREMENT STEP
    bool gpsOk = gps.time > 0;

    float gpsX = 0, gpsY = 0;
    float jump = 0;

    if (gpsOk) {
        gpsToXY(gps.lat, gps.lon, gpsX, gpsY);

        jump = distance(x, y, gpsX, gpsY);

        bool spoof = detectSpoof(gps, jump, gps.speed);

        if (spoof) {
            gpsTrusted = false;
            gpsBanUntil = now + 2000;
        }

        if (now > gpsBanUntil) {
            gpsTrusted = true;
        }
    }

    state.gpsValid = gpsOk;
    state.gpsTrusted = gpsTrusted;

    // FUSION (simple Kalman style)
    if (gpsOk && gpsTrusted) {

        // correction
        x = 0.85f * x + 0.15f * gpsX;
        y = 0.85f * y + 0.15f * gpsY;

        // velocity correction
        vx = 0.85f * vx + 0.15f * gps.speed;
    }

    // OUTPUT METRICS
    float dist = sqrtf(x*x + y*y);

    state.distance = dist;
    state.currentSpeed = sqrtf(vx*vx + vy*vy);

    float t = state.totalTime / 1000.0f;
    state.avgSpeed = (t > 0) ? dist / t : 0;

    if (state.currentSpeed > 0.1f)
        state.pace500 = 500.0f / state.currentSpeed;

    // STROKES (IMU peak detect)
    static bool stroke = false;

    if (imu.az > 12.0f && !stroke) {
        state.strokeCount++;
        stroke = true;
    }

    if (imu.az < 9.5f) {
        stroke = false;
    }

    state.valid = true;
}

const NavigationState& NavCore::get() const {
    return state;
}