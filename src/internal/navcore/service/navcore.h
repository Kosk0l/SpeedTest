#pragma once

#include <cstdint>
#include "internal/gps/domain/GpsData.h"
#include "internal/imu/domain/ImuData.h"
#include "internal/navcore/domain/NavigationState.h"

// Дистанция и скорость — только по GPS (доплеровская скорость приёмника).
// IMU — только детектор гребков.
class NavCore {
private:
    NavigationState state;

    // Тайминг
    bool started = false;
    uint32_t startTime = 0;

    // ================= GPS: дистанция и скорость =================

    uint32_t lastGpsStamp = 0;      // gps.time последнего обработанного пакета
    uint32_t lastGoodFixTime = 0;   // millis() последнего годного фикса

    // Доверие к GPS с гистерезисом
    bool gpsTrusted = false;
    uint8_t goodFixStreak = 0;
    uint8_t badFixStreak = 0;

    // Последний фикс, учтённый в дистанции (опорная точка)
    bool hasLastFix = false;
    double lastLat = 0.0, lastLon = 0.0;
    float lastHAcc = 0.0f;
    uint32_t lastITow = 0;

    // Детектор движения: последние фиксы для усреднения вектора скорости
    struct MotionSample {
        uint16_t dtMs;
        float step;           // путь за шаг (м)
        float velN, velE;     // вектор скорости (м/с)
    };
    static constexpr uint8_t MOTION_RING_SIZE = 32;  // окно 2 с влезает и при 10+ Гц
    MotionSample motionRing[MOTION_RING_SIZE] = {};
    uint8_t motionHead = 0;
    uint8_t motionCount = 0;
    bool moving = false;
    float lastAlongSpeed = 0.0f;    // скорость вдоль курса на прошлой эпохе (м/с)
    float restNoiseVar = 0.0f;      // средний квадрат скорости на стоянке (м2/с2)

    // Предварительный старт: путь копится отдельно, пока координаты не подтвердят смещение
    bool provisional = false;
    double provisionalLat = 0.0, provisionalLon = 0.0;
    float provisionalHAcc = 0.0f;
    double provisionalDistance = 0.0;
    uint32_t provisionalMs = 0;
    double provisionalN = 0.0, provisionalE = 0.0;  // смещение по интегралу скорости (м)

    double totalDistance = 0.0;
    uint32_t movingTimeMs = 0;
    float smoothedSpeed = 0.0f;

    // Время UTC с GPS
    uint32_t unixBase = 0;
    uint32_t unixBaseMillis = 0;

    // ================= IMU: гребки =================

    uint32_t lastImuStamp = 0;
    bool imuInitialized = false;
    uint32_t imuStartTime = 0;
    uint16_t imuSamples = 0;

    float gravX = 0.0f, gravY = 0.0f, gravZ = 0.0f;   // оценка вектора гравитации
    float cov[6] = {0};                               // ковариация горизонтального ускорения: xx xy xz yy yz zz
    float axisX = 1.0f, axisY = 0.0f, axisZ = 0.0f;   // продольная ось лодки (главная компонента)
    float surge = 0.0f;                               // продольное ускорение, сглаженное
    float cycleMax = 0.0f, cycleMin = 0.0f;           // размах текущего цикла

    // Типичная амплитуда гребка: медиана последних циклов
    static constexpr uint8_t AMP_HISTORY_SIZE = 5;
    float ampHistory[AMP_HISTORY_SIZE] = {0};
    uint8_t ampHead = 0;
    uint8_t ampCount = 0;

    bool strokeHigh = false;
    uint32_t lastTriggerTime = 0;   // последнее срабатывание триггера, любое

    // Ритм: гребок засчитывается, только если повторяется с ровным периодом
    static constexpr uint8_t RHYTHM_CANDIDATES = 5;
    uint32_t candidateTime[RHYTHM_CANDIDATES] = {0};
    float candidateAmp[RHYTHM_CANDIDATES] = {0};
    uint8_t candidateCount = 0;
    bool rhythmLocked = false;
    float strokePeriodMs = 0.0f;
    uint32_t lastAcceptedTime = 0;
    bool dirtySinceAccepted = false;  // были лишние срабатывания в текущем цикле
    uint8_t dirtyCycles = 0;          // циклов подряд с лишними срабатываниями

    uint32_t lastStrokeTime = 0;    // последний засчитанный гребок
    float strokeRate = 0.0f;

    // Гребки, пойманные на стоянке: ждут подтверждения стартом по GPS
    static constexpr uint8_t MAX_PENDING_STROKES = 8;
    uint32_t pendingStrokes[MAX_PENDING_STROKES] = {0};
    uint8_t pendingStrokeCount = 0;

    // Вспомогательные методы
    void processGps(const GpsData& gps, uint32_t now);
    bool isFixGood(const GpsData& gps) const;
    void rememberFix(const GpsData& gps);
    void updateMotion(const GpsData& gps, uint32_t dtMs);
    void processImu(const ImuData& imu);
    float medianAmplitude() const;
    void resetRhythm();
    void onStrokeTrigger(uint32_t t, float cycleAmp);
    void onStrokeDetected(uint32_t t);
    void countStroke(uint32_t t);
    void updatePendingStrokes(uint32_t now);
    static void offsetNE(double lat1, double lon1, double lat2, double lon2, double& north, double& east);
    static double distanceBetween(double lat1, double lon1, double lat2, double lon2);

public:
    // Constructor
    NavCore();

    // main method - calculation
    // Можно вызывать на каждом loop(): новые пакеты определяются по меткам времени
    void update(const GpsData& gps, const ImuData& imu);
    // Return state
    const NavigationState& get() const;
};
