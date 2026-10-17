#include "internal/navcore/service/navcore.h"
#include <Arduino.h>
#include <math.h>

// ============================================================================
// ФИЗИЧЕСКИЕ КОНСТАНТЫ И ПОРОГИ
// ============================================================================

// Лимиты для академической гребли
constexpr float MAX_ROWING_SPEED = 7.0f;    // м/с — быстрее лодка не ходит
constexpr float GPS_MAX_SPEED    = 10.5f;   // м/с — выше считаем выбросом GPS

// Качество фикса. Войти в доверие строже, чем остаться в нём (гистерезис)
constexpr float   GPS_HACC_ENTER = 5.0f;    // м
constexpr float   GPS_HACC_STAY  = 8.0f;    // м
constexpr float   GPS_SACC_ENTER = 0.6f;    // м/с
constexpr float   GPS_SACC_STAY  = 1.0f;    // м/с
constexpr uint8_t GPS_SATS_ENTER = 6;
constexpr uint8_t GPS_SATS_STAY  = 5;
constexpr uint8_t GPS_TRUST_ENTER_FIXES = 5;   // годных фиксов подряд (1 с при 5 Гц)
constexpr uint8_t GPS_TRUST_EXIT_FIXES  = 3;   // плохих фиксов подряд
constexpr uint32_t GPS_TIMEOUT_MS       = 2000; // нет годных фиксов — GPS потерян

// Накопление дистанции
constexpr uint32_t GPS_WEEK_MS                = 604800000;
constexpr uint32_t GPS_MAX_INTEGRATION_GAP_MS = 3000;    // до этого разрыва интегрируем скорость
constexpr uint32_t GPS_MAX_BRIDGE_MS          = 300000;  // до этого — мостим хордой по координатам
constexpr float    GPS_GAP_NOISE_FACTOR       = 2.0f;    // хорда меньше 2*(hAcc1+hAcc2) — шум позиции

// Детектор движения по доплеровской скорости
constexpr uint32_t MOTION_WINDOW_MS = 2000;    // окно усреднения вектора скорости
constexpr float    MOVE_START_SPEED = 1.0f;    // м/с — снос течением и шум ниже этого
constexpr float    MOVE_SACC_FACTOR = 2.0f;    // и не ниже 2*sAcc
constexpr float    MOVE_NOISE_FACTOR = 2.5f;   // и не ниже 2.5*RMS шума скорости на стоянке
constexpr float    REST_NOISE_TAU    = 20.0f;  // с — усреднение шума скорости на стоянке
constexpr float    MOVE_STOP_SPEED  = 0.5f;    // м/с
constexpr float    MOVE_MIN_DISPLACEMENT    = 10.0f;  // м — старт подтверждается смещением координат
constexpr float    MOVE_DISPLACEMENT_FACTOR = 1.5f;   // и не меньше 1.5*(hAcc1+hAcc2)
constexpr float    MOVE_MATCH_MIN           = 5.0f;   // м — допустимое расхождение смещений по координатам и по скорости
constexpr float    MOVE_MATCH_RATIO         = 0.2f;   // плюс доля от пройденного
constexpr uint32_t MOVE_CONFIRM_TIMEOUT_MS  = 20000;  // не подтвердилось — это был шум скорости

// Вывод скорости
constexpr float SPEED_SMOOTH_TAU = 2.0f;       // с — сглаживание колебаний скорости внутри гребка
constexpr float MIN_PACE_SPEED   = 1.0f;       // м/с — медленнее темп не показываем (8:20)

// Детектор гребков
constexpr float    GRAVITY_TAU            = 4.0f;    // с — ФНЧ гравитации (период гребка 1–3.5 с)
constexpr float    AXIS_TAU               = 5.0f;    // с — усреднение ковариации для оси лодки
constexpr float    SURGE_TAU              = 0.05f;   // с — сглаживание вибраций (~3 Гц)
constexpr float    STROKE_MIN_ACCEL       = 0.8f;    // м/с2 — порог не ниже этого (шум, качка)
constexpr float    STROKE_THRESHOLD_RATIO = 0.3f;    // порог — доля от типичной амплитуды гребка
constexpr uint32_t STROKE_MIN_INTERVAL_MS = 1000;    // 60 гребков/мин — чаще это не гребля
constexpr uint32_t STROKE_MAX_INTERVAL_MS = 5000;    // 12 гребков/мин — реже ритм потерян
constexpr float    STROKE_RHYTHM_RATIO    = 1.3f;    // интервалы различаются не больше чем на 30%
constexpr float    STROKE_AMP_RATIO       = 2.5f;    // и амплитуды — не больше чем в 2.5 раза
constexpr float    STROKE_PERIOD_ALPHA    = 0.3f;    // сглаживание периода гребка
constexpr float    STROKE_RATE_ALPHA      = 0.4f;    // сглаживание темпа
constexpr uint32_t STROKE_PENDING_WINDOW_MS = 20000; // гребок на стоянке ждёт подтверждения стартом
constexpr float    IMU_MAX_DT             = 0.1f;    // с — ограничение шага при пропусках
constexpr uint32_t IMU_TIMEOUT_MS         = 500;
constexpr uint32_t IMU_WARMUP_MS          = 2000;    // после включения фильтры устаиваются


NavCore::NavCore() {
    state = {};
}

void NavCore::offsetNE(double lat1, double lon1, double lat2, double lon2, double& north, double& east) {
    // Равнопромежуточная проекция: на дистанциях гребли погрешность — миллиметры.
    // Считаем в double: float на широте 55° даёт шаг ~0.4 м
    const double R = 6371000.0;
    const double DEG = M_PI / 180.0;

    north = R * (lat2 - lat1) * DEG;
    east  = R * (lon2 - lon1) * DEG * cos(0.5 * (lat1 + lat2) * DEG);
}

double NavCore::distanceBetween(double lat1, double lon1, double lat2, double lon2) {
    double north, east;
    offsetNE(lat1, lon1, lat2, lon2, north, east);
    return sqrt(north * north + east * east);
}


// ============================================================================
// GPS: ДИСТАНЦИЯ И СКОРОСТЬ
// ============================================================================

bool NavCore::isFixGood(const GpsData& gps) const {
    if (!gps.fixOk) return false;
    if (gps.fixType != 3 && gps.fixType != 4) return false;  // только 3D (и GNSS+DR)
    if (gps.lat == 0.0 && gps.lon == 0.0) return false;
    if (gps.speed < 0.0f || gps.speed > GPS_MAX_SPEED) return false;

    // Если GPS уже в доверии — пороги мягче, чтобы не мигать на границе
    float maxHAcc   = gpsTrusted ? GPS_HACC_STAY : GPS_HACC_ENTER;
    float maxSAcc   = gpsTrusted ? GPS_SACC_STAY : GPS_SACC_ENTER;
    uint8_t minSats = gpsTrusted ? GPS_SATS_STAY : GPS_SATS_ENTER;

    return gps.hAcc <= maxHAcc && gps.sAcc <= maxSAcc && gps.satellites >= minSats;
}

void NavCore::rememberFix(const GpsData& gps) {
    hasLastFix = true;
    lastLat   = gps.lat;
    lastLon   = gps.lon;
    lastHAcc  = gps.hAcc;
    lastITow  = gps.iTow;
}

void NavCore::updateMotion(const GpsData& gps, uint32_t dtMs) {
    uint8_t current = motionHead;
    motionRing[current] = { (uint16_t)dtMs, 0.0f, gps.velN, gps.velE };
    motionHead = (motionHead + 1) % MOTION_RING_SIZE;
    if (motionCount < MOTION_RING_SIZE) motionCount++;

    // Средний вектор скорости за окно. На стоянке шум смотрит в случайные стороны
    // и гасится, в движении — складывается. По модулю скорости так нельзя:
    // его шум всегда в плюс, и стоящая лодка «едет»
    uint32_t windowMs = 0;
    float sumN = 0.0f, sumE = 0.0f;
    double windowStep = 0.0;
    for (uint8_t i = 0; i < motionCount && windowMs < MOTION_WINDOW_MS; i++) {
        const MotionSample& m = motionRing[(motionHead + MOTION_RING_SIZE - 1 - i) % MOTION_RING_SIZE];
        windowMs   += m.dtMs;
        sumN       += m.velN * m.dtMs;
        sumE       += m.velE * m.dtMs;
        windowStep += m.step;
    }
    bool windowFull = windowMs >= MOTION_WINDOW_MS;
    float sumNorm = sqrtf(sumN * sumN + sumE * sumE);
    float coherentSpeed = (windowMs > 0) ? sumNorm / windowMs : 0.0f;

    // Скорость вдоль курса — проекция на средний вектор: поперечный шум её не завышает
    float alongSpeed = (sumNorm > 1e-6f) ? (gps.velN * sumN + gps.velE * sumE) / sumNorm : 0.0f;
    alongSpeed = fminf(fmaxf(alongSpeed, 0.0f), MAX_ROWING_SPEED);

    // Трапеция между эпохами
    double step = 0.5 * (lastAlongSpeed + alongSpeed) * (dtMs / 1000.0);
    lastAlongSpeed = alongSpeed;
    motionRing[current].step = (float)step;
    windowStep += step;

    float dt = dtMs / 1000.0f;

    if (!moving && !provisional) {
        // Шум скорости, который реально видим на стоянке: приёмник может занижать sAcc.
        // Только когда окно явно говорит «стоим» — иначе разгон поднимет порог старта
        if (windowFull && coherentSpeed < MOVE_STOP_SPEED) {
            restNoiseVar += (dt / (REST_NOISE_TAU + dt)) * (gps.velN * gps.velN + gps.velE * gps.velE - restNoiseVar);
        }

        float startSpeed = fmaxf(MOVE_START_SPEED,
                           fmaxf(MOVE_SACC_FACTOR * gps.sAcc, MOVE_NOISE_FACTOR * sqrtf(restNoiseVar)));
        if (windowFull && coherentSpeed > startSpeed) {
            // Предварительный старт: путь из окна — это разгон, но засчитаем его,
            // только когда координаты подтвердят смещение
            provisional = true;
            provisionalLat  = gps.lat;
            provisionalLon  = gps.lon;
            provisionalHAcc = gps.hAcc;
            provisionalDistance = windowStep;
            provisionalMs   = windowMs;
            provisionalN = provisionalE = 0.0;
            smoothedSpeed   = alongSpeed;
            motionCount = 0;  // окно учтено, дальше копим заново
        }
        return;
    }

    if (windowFull && coherentSpeed < MOVE_STOP_SPEED) {
        // Остановка. Если старт так и не подтвердился — путь был шумом, он не засчитан
        moving = false;
        provisional = false;
        smoothedSpeed = 0.0f;
        motionCount = 0;
        return;
    }

    smoothedSpeed += (dt / (SPEED_SMOOTH_TAU + dt)) * (alongSpeed - smoothedSpeed);

    if (provisional) {
        provisionalDistance += step;
        provisionalMs       += dtMs;
        provisionalN        += gps.velN * dt;   // смещение, проинтегрированное из скорости
        provisionalE        += gps.velE * dt;

        // Гребля уводит координаты на десятки метров, и туда же, куда показывает скорость.
        // Шум скорости и дрейф координат так не согласуются
        double north, east;
        offsetNE(provisionalLat, provisionalLon, gps.lat, gps.lon, north, east);
        double moved   = sqrt(north * north + east * east);
        double needed  = fmax(MOVE_MIN_DISPLACEMENT, MOVE_DISPLACEMENT_FACTOR * (provisionalHAcc + gps.hAcc));
        double integrated = sqrt(provisionalN * provisionalN + provisionalE * provisionalE);
        double mismatch = sqrt((north - provisionalN) * (north - provisionalN) + (east - provisionalE) * (east - provisionalE));
        double allowed  = fmax(MOVE_MATCH_MIN, provisionalHAcc + gps.hAcc) + MOVE_MATCH_RATIO * integrated;

        if (moved >= needed && mismatch <= allowed) {
            moving = true;
            provisional = false;
            totalDistance += provisionalDistance;
            movingTimeMs  += provisionalMs;
        } else if (provisionalMs > MOVE_CONFIRM_TIMEOUT_MS) {
            provisional = false;
            smoothedSpeed = 0.0f;
            motionCount = 0;
        }
        return;
    }

    totalDistance += step;
    movingTimeMs  += dtMs;
}

void NavCore::processGps(const GpsData& gps, uint32_t now) {
    state.satellites = gps.satellites;
    state.hAcc = gps.hAcc;
    if (gps.unixTime != 0) {
        unixBase = gps.unixTime;
        unixBaseMillis = gps.time;
    }

    // 1. Качество фикса
    if (!isFixGood(gps)) {
        goodFixStreak = 0;
        if (badFixStreak < 255) badFixStreak++;
        if (gpsTrusted && badFixStreak >= GPS_TRUST_EXIT_FIXES) {
            gpsTrusted = false;
        }
        // Плохой фикс не трогает ни дистанцию, ни опорную точку
        return;
    }

    badFixStreak = 0;
    if (goodFixStreak < 255) goodFixStreak++;
    lastGoodFixTime = now;

    if (!gpsTrusted) {
        // Прогрев: первые фиксы после потери сигнала прыгают
        if (goodFixStreak < GPS_TRUST_ENTER_FIXES) return;
        gpsTrusted = true;
    }

    // 2. Первый фикс — только опорная точка
    if (!hasLastFix) {
        rememberFix(gps);
        return;
    }

    // Время между эпохами по часам приёмника (с переходом через конец недели)
    uint32_t dtMs = (gps.iTow + GPS_WEEK_MS - lastITow) % GPS_WEEK_MS;
    if (dtMs == 0) return;  // та же эпоха

    float speed = fminf(gps.speed, MAX_ROWING_SPEED);

    if (dtMs <= GPS_MAX_INTEGRATION_GAP_MS) {
        // 3. Интегрируем доплеровскую скорость.
        //    Шум координат на дистанцию не влияет — в этом весь смысл
        updateMotion(gps, dtMs);
    } else if (dtMs <= GPS_MAX_BRIDGE_MS) {
        // 4. Долгий провал сигнала: что было в нём, неизвестно,
        //    надёжно только смещение между опорной и текущей точкой
        //    Стояли до провала и стоим после — прыжок координат при захвате спутников не путь
        double chord = distanceBetween(lastLat, lastLon, gps.lat, gps.lon);
        double noise = GPS_GAP_NOISE_FACTOR * (lastHAcc + gps.hAcc);
        double maxStep = MAX_ROWING_SPEED * (dtMs / 1000.0);
        bool wasMoving = moving || provisional || speed >= MOVE_START_SPEED;
        bool displaced = wasMoving && chord > noise && chord <= maxStep;

        if (displaced) {
            totalDistance += chord;
            movingTimeMs  += dtMs;
            if (provisional) {
                totalDistance += provisionalDistance;
                movingTimeMs  += provisionalMs;
            }
        }
        moving = (moving || displaced) && speed >= MOVE_STOP_SPEED;
        provisional = false;
        smoothedSpeed = moving ? speed : 0.0f;
        lastAlongSpeed = smoothedSpeed;
        motionCount = 0;  // скорости до провала устарели
    } else {
        // 5. Слишком долго без сигнала — начинаем с новой опорной точки
        moving = false;
        provisional = false;
        smoothedSpeed = 0.0f;
        lastAlongSpeed = 0.0f;
        motionCount = 0;
    }

    rememberFix(gps);
}


// ============================================================================
// IMU: ГРЕБКИ
// ============================================================================

void NavCore::processImu(const ImuData& imu) {
    uint32_t prevStamp = lastImuStamp;
    lastImuStamp = imu.time;

    float gNorm;
    if (!imuInitialized) {
        gNorm = sqrtf(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
        if (gNorm < 1.0f) return;  // мусор с датчика

        gravX = imu.ax;
        gravY = imu.ay;
        gravZ = imu.az;
        imuSamples = 1;
        imuStartTime = imu.time;

        // Начальная ось — горизонтальная проекция той оси датчика,
        // что меньше всего смотрит вдоль гравитации
        float ux = gravX / gNorm, uy = gravY / gNorm, uz = gravZ / gNorm;
        float ex = 0.0f, ey = 0.0f, ez = 0.0f;
        if (fabsf(ux) <= fabsf(uy) && fabsf(ux) <= fabsf(uz)) ex = 1.0f;
        else if (fabsf(uy) <= fabsf(uz)) ey = 1.0f;
        else ez = 1.0f;
        float ev = ex * ux + ey * uy + ez * uz;
        ex -= ev * ux; ey -= ev * uy; ez -= ev * uz;
        float eNorm = sqrtf(ex * ex + ey * ey + ez * ez);
        axisX = ex / eNorm; axisY = ey / eNorm; axisZ = ez / eNorm;

        imuInitialized = true;
        return;
    }

    float dt = (imu.time - prevStamp) / 1000.0f;
    if (dt <= 0.0f) return;
    if (dt > IMU_MAX_DT) dt = IMU_MAX_DT;

    // 1. Гравитация — медленный ФНЧ. Всё, что быстрее, — динамика лодки.
    //    Гироскоп не используем: крен на частоте гребка мал, а дрейф нуля — лишняя ошибка.
    //    Первые сэмплы усредняем накопительно, чтобы не стартовать с одного шумного
    if (imuSamples < 65535) imuSamples++;
    float kG = fmaxf(dt / (GRAVITY_TAU + dt), 1.0f / imuSamples);
    gravX += kG * (imu.ax - gravX);
    gravY += kG * (imu.ay - gravY);
    gravZ += kG * (imu.az - gravZ);

    gNorm = sqrtf(gravX * gravX + gravY * gravY + gravZ * gravZ);
    if (gNorm < 1.0f) return;
    float ux = gravX / gNorm, uy = gravY / gNorm, uz = gravZ / gNorm;

    // 2. Динамическое ускорение без вертикальной составляющей
    float dx = imu.ax - gravX, dy = imu.ay - gravY, dz = imu.az - gravZ;
    float dv = dx * ux + dy * uy + dz * uz;
    float hx = dx - dv * ux, hy = dy - dv * uy, hz = dz - dv * uz;

    // 3. Продольная ось — направление, где горизонтальное ускорение сильнее всего
    //    (главная компонента). Не зависит от того, как закреплён прибор
    float kC = dt / (AXIS_TAU + dt);
    cov[0] += kC * (hx * hx - cov[0]);
    cov[1] += kC * (hx * hy - cov[1]);
    cov[2] += kC * (hx * hz - cov[2]);
    cov[3] += kC * (hy * hy - cov[3]);
    cov[4] += kC * (hy * hz - cov[4]);
    cov[5] += kC * (hz * hz - cov[5]);

    // Одна итерация степенного метода на сэмпл: ось плавно сходится и не меняет знак
    float nx = cov[0] * axisX + cov[1] * axisY + cov[2] * axisZ;
    float ny = cov[1] * axisX + cov[3] * axisY + cov[4] * axisZ;
    float nz = cov[2] * axisX + cov[4] * axisY + cov[5] * axisZ;
    float nv = nx * ux + ny * uy + nz * uz;  // держим ось горизонтальной
    nx -= nv * ux; ny -= nv * uy; nz -= nv * uz;
    float nNorm = sqrtf(nx * nx + ny * ny + nz * nz);
    if (nNorm > 1e-9f) {
        axisX = nx / nNorm; axisY = ny / nNorm; axisZ = nz / nNorm;
    }

    // 4. Продольное ускорение, сглаженное от вибраций
    float s = hx * axisX + hy * axisY + hz * axisZ;
    surge += (dt / (SURGE_TAU + dt)) * (s - surge);

    // 5. Порог — доля от медианной амплитуды последних гребков:
    //    одиночный удар его не задирает, лёгкая гребля не теряется
    cycleMax = fmaxf(cycleMax, surge);
    cycleMin = fminf(cycleMin, surge);
    float threshold = fmaxf(STROKE_MIN_ACCEL, STROKE_THRESHOLD_RATIO * medianAmplitude());

    // Пока фильтры не устоялись, гребки не ищем
    if (imu.time - imuStartTime < IMU_WARMUP_MS) return;

    // 6. Триггер Шмитта: одно срабатывание на полный цикл разгон -> торможение.
    //    Знак оси не важен: на цикл приходится ровно один переход вверх
    if (!strokeHigh) {
        if (surge > threshold) {
            strokeHigh = true;
            float cycleAmp = 0.5f * (cycleMax - cycleMin);
            cycleMax = cycleMin = surge;
            onStrokeTrigger(imu.time, cycleAmp);
        }
    } else if (surge < -threshold) {
        strokeHigh = false;
    }
}

float NavCore::medianAmplitude() const {
    if (ampCount == 0) return 0.0f;
    float sorted[AMP_HISTORY_SIZE];
    for (uint8_t i = 0; i < ampCount; i++) {
        float v = ampHistory[i];
        uint8_t j = i;
        for (; j > 0 && sorted[j - 1] > v; j--) sorted[j] = sorted[j - 1];
        sorted[j] = v;
    }
    return sorted[ampCount / 2];
}

void NavCore::resetRhythm() {
    rhythmLocked = false;
    candidateCount = 0;
    dirtyCycles = 0;
    dirtySinceAccepted = false;
}

void NavCore::onStrokeTrigger(uint32_t t, float cycleAmp) {
    // 7. Проверка ритма. Гребля — это серия циклов с ровным периодом 1–5 с.
    //    Удар, тряска, шаги так не выглядят
    uint32_t sinceTrigger = lastTriggerTime ? t - lastTriggerTime : UINT32_MAX;
    lastTriggerTime = t;

    if (sinceTrigger >= STROKE_MIN_INTERVAL_MS && sinceTrigger <= STROKE_MAX_INTERVAL_MS) {
        ampHistory[ampHead] = cycleAmp;
        ampHead = (ampHead + 1) % AMP_HISTORY_SIZE;
        if (ampCount < AMP_HISTORY_SIZE) ampCount++;
    }

    if (rhythmLocked) {
        // Следующий гребок ждём через период ±30%
        float ratio = (t - lastAcceptedTime) / strokePeriodMs;

        if (ratio < 1.0f / STROKE_RHYTHM_RATIO) {
            // Раньше срока — лишнее срабатывание внутри цикла: всплеск, удар
            dirtySinceAccepted = true;
            return;
        }

        if (ratio <= STROKE_RHYTHM_RATIO && t - lastAcceptedTime <= STROKE_MAX_INTERVAL_MS) {
            dirtyCycles = dirtySinceAccepted ? dirtyCycles + 1 : 0;
            dirtySinceAccepted = false;

            if (dirtyCycles < 2) {
                strokePeriodMs += STROKE_PERIOD_ALPHA * ((t - lastAcceptedTime) - strokePeriodMs);
                lastAcceptedTime = t;
                onStrokeDetected(t);
                return;
            }
            // Два цикла подряд с лишними срабатываниями — это уже не гребля (шаги, тряска)
        }

        // Ритм сбился (пауза, пропуск, не гребля). Срабатывание открывает новую серию,
        // при повторном захвате она засчитается целиком — гребки не теряются
        resetRhythm();
    }

    // Поиск ритма: копим серию срабатываний
    if (sinceTrigger < STROKE_MIN_INTERVAL_MS || sinceTrigger > STROKE_MAX_INTERVAL_MS) {
        candidateCount = 0;  // слишком часто или слишком редко для гребли — серия заново
    }
    if (candidateCount == RHYTHM_CANDIDATES) {
        for (uint8_t i = 1; i < RHYTHM_CANDIDATES; i++) {
            candidateTime[i - 1] = candidateTime[i];
            candidateAmp[i - 1]  = candidateAmp[i];
        }
        candidateCount--;
    }
    candidateTime[candidateCount] = t;
    candidateAmp[candidateCount]  = cycleAmp;
    candidateCount++;

    if (candidateCount < RHYTHM_CANDIDATES) return;

    // Серия из RHYTHM_CANDIDATES срабатываний: соседние интервалы ровные,
    // амплитуды циклов внутри серии похожи. Случайные удары так не ложатся
    float ampMin = candidateAmp[1], ampMax = candidateAmp[1];
    for (uint8_t i = 1; i < RHYTHM_CANDIDATES; i++) {
        ampMin = fminf(ampMin, candidateAmp[i]);
        ampMax = fmaxf(ampMax, candidateAmp[i]);
        if (i >= 2) {
            float prev = (float)(candidateTime[i - 1] - candidateTime[i - 2]);
            float curr = (float)(candidateTime[i] - candidateTime[i - 1]);
            if (curr > prev * STROKE_RHYTHM_RATIO || prev > curr * STROKE_RHYTHM_RATIO) return;
        }
    }
    if (ampMax > ampMin * STROKE_AMP_RATIO) return;

    // Ритм пойман — засчитываем всю серию, первые гребки не теряются
    rhythmLocked = true;
    strokePeriodMs = (float)(candidateTime[RHYTHM_CANDIDATES - 1] - candidateTime[RHYTHM_CANDIDATES - 2]);
    lastAcceptedTime = candidateTime[RHYTHM_CANDIDATES - 1];
    dirtyCycles = 0;
    dirtySinceAccepted = false;
    for (uint8_t i = 0; i < RHYTHM_CANDIDATES; i++) onStrokeDetected(candidateTime[i]);
    candidateCount = 0;
}

void NavCore::onStrokeDetected(uint32_t t) {
    if (gpsTrusted && !moving) {
        // GPS уверенно говорит «стоим» — это может быть качка у причала.
        // Откладываем: если следом подтвердится старт, это были первые гребки
        if (pendingStrokeCount == MAX_PENDING_STROKES) {
            for (uint8_t i = 1; i < MAX_PENDING_STROKES; i++) pendingStrokes[i - 1] = pendingStrokes[i];
            pendingStrokeCount--;
        }
        pendingStrokes[pendingStrokeCount++] = t;
        return;
    }
    countStroke(t);
}

void NavCore::countStroke(uint32_t t) {
    state.strokeCount++;

    uint32_t interval = t - lastStrokeTime;
    if (lastStrokeTime != 0 && interval <= STROKE_MAX_INTERVAL_MS) {
        float spm = 60000.0f / interval;
        strokeRate = (strokeRate > 0.0f)
            ? strokeRate + STROKE_RATE_ALPHA * (spm - strokeRate)
            : spm;
    }
    lastStrokeTime = t;
}

void NavCore::updatePendingStrokes(uint32_t now) {
    if (pendingStrokeCount == 0) return;

    // Старт подтверждён (или GPS больше не уверен в стоянке) — засчитываем свежие
    bool release = !(gpsTrusted && !moving);

    uint8_t kept = 0;
    for (uint8_t i = 0; i < pendingStrokeCount; i++) {
        if (now - pendingStrokes[i] > STROKE_PENDING_WINDOW_MS) continue;  // устарел — была качка
        if (release) countStroke(pendingStrokes[i]);
        else pendingStrokes[kept++] = pendingStrokes[i];
    }
    pendingStrokeCount = kept;
}


// ============================================================================
// ГЛАВНЫЙ ЦИКЛ
// ============================================================================

void NavCore::update(const GpsData& gps, const ImuData& imu) {
    uint32_t now = millis();

    if (!started) {
        started = true;
        startTime = now;
    }
    state.totalTime = now - startTime;

    // Обрабатываем только новые пакеты: update() зовётся на каждом loop()
    if (gps.time != 0 && gps.time != lastGpsStamp) {
        lastGpsStamp = gps.time;
        processGps(gps, now);
    }
    updatePendingStrokes(now);  // до IMU: гребки засчитываются по порядку
    if (imu.time != 0 && imu.time != lastImuStamp) {
        processImu(imu);
    }

    // Таймауты
    if (gpsTrusted && now - lastGoodFixTime > GPS_TIMEOUT_MS) {
        gpsTrusted = false;
        goodFixStreak = 0;
    }
    if (lastStrokeTime != 0 && now - lastStrokeTime > STROKE_MAX_INTERVAL_MS) {
        strokeRate = 0.0f;
    }
    if (lastTriggerTime != 0 && now - lastTriggerTime > STROKE_MAX_INTERVAL_MS) {
        // Давно нет срабатываний: ритм и типичная амплитуда устарели
        resetRhythm();
        lastTriggerTime = 0;
        ampCount = 0;
    }

    // Дистанция и скорость
    state.distance   = (float)totalDistance;
    state.movingTime = movingTimeMs;
    state.avgSpeed   = (movingTimeMs > 0) ? (float)(totalDistance / (movingTimeMs / 1000.0)) : 0.0f;

    float speed = (gpsTrusted && (moving || provisional)) ? smoothedSpeed : 0.0f;
    state.currentSpeed = speed;
    state.pace500 = (speed >= MIN_PACE_SPEED) ? 500.0f / speed : 0.0f;

    // Гребки
    state.strokeRate = strokeRate;
    state.inStroke   = strokeHigh;

    // Время: между пакетами GPS досчитываем по millis()
    state.utcTime = (unixBase != 0) ? unixBase + (now - unixBaseMillis) / 1000 : 0;

    // Статусы
    bool gpsFresh = gps.time != 0 && now - gps.time < GPS_TIMEOUT_MS;
    if (!gpsFresh) state.satellites = 0;  // пакеты не идут — не показываем устаревшее
    state.gpsValid   = gpsFresh && gps.fixOk;
    state.gpsTrusted = gpsTrusted;
    state.imuValid   = imuInitialized && now - lastImuStamp < IMU_TIMEOUT_MS;
    state.valid      = true;
    state.isMoving   = moving;
}

const NavigationState& NavCore::get() const {
    return state;
}
