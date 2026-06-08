#include "internal/navcore/service/navcore.h"
#include <Arduino.h>
#include <math.h>

// ФИЗИЧЕСКИЕ КОНСТАНТЫ И ПОРОГИ:

// Лимиты для академической гребли
// Мировой рекорд одиночки ~5.5 м/с (20 км/ч), восьмёрки ~6.5 м/с
constexpr float MAX_ROWING_SPEED    = 7.0f;    // м/с — жёсткий потолок
constexpr float MIN_SPEED_THRESHOLD = 0.15f;   // м/с — ниже считаем покой
constexpr float GRAVITY             = 9.81f;   // м/с²

// GPS доверие
constexpr float GPS_QUALITY_DECAY   = 0.98f;   // деградация качества/тик без GPS
constexpr float GPS_MAX_JUMP        = 25.0f;   // м — скачок позиции = спуффинг
constexpr float GPS_SPEED_DEVIATION = 2.5f;    // м/с — расхождение с IMU

// Детектор гребков (адаптивные пороги подстраиваются под ритм)
constexpr float STROKE_THRESHOLD_HIGH = 11.5f; // м/с² — начало гребка (catch)
constexpr float STROKE_THRESHOLD_LOW  = 9.8f;  // м/с² — конец гребка (recovery)
constexpr uint32_t STROKE_MIN_INTERVAL = 700;  // мс (~85 гребков/мин максимум)
constexpr uint32_t STROKE_MAX_INTERVAL = 3500; // мс (~17 гребков/мин минимум)

// Режимы работы
constexpr float GPS_QUALITY_EXCELLENT = 0.7f;
constexpr float GPS_QUALITY_POOR      = 0.3f;
constexpr float GPS_QUALITY_DEAD      = 0.1f;   // ниже этого — GPS полностью игнорируем

// Damping в зависимости от режима
constexpr float DAMPING_EXCELLENT = 0.999f;  // почти не давим
constexpr float DAMPING_POOR      = 0.95f;   // среднее подавление
constexpr float DAMPING_DEAD      = 0.85f;   // жёсткое подавление

// ZUPT пороги
constexpr float ZUPT_ACCEL_THRESHOLD_GOOD = 0.6f;   // когда GPS есть
constexpr float ZUPT_ACCEL_THRESHOLD_DEAD = 1.2f;   // когда GPS нет — более агрессивный
constexpr float ZUPT_VELOCITY_THRESHOLD  = 0.08f;   // м/с — ниже этого обнуляем

// Bias калибровка
constexpr float BIAS_ALPHA_FAST   = 0.05f;  // когда GPS нет
constexpr float BIAS_ALPHA_SLOW   = 0.01f;  // когда GPS есть

// Velocity clamping — если скорость очень мала и нет GPS, обнуляем
constexpr float VELOCITY_CLAMP_THRESHOLD = 0.05f;  // м/с

NavCore::NavCore() {
    state = {};
}

// ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ:

float NavCore::deg2rad(float d) {
    return d * 0.01745329251f;
}

// Плоская проекция WGS84 → локальные XY (метод equirectangular approximation)
// Точность достаточна для дистанций < 5 км (дистанция гребли 2 км)
void NavCore::gpsToXY(double lat, double lon, float& xOut, float& yOut) {
    if (!originSet) {
        lat0 = lat;
        lon0 = lon;
        originSet = true;
        xOut = yOut = 0.0f;
        return;
    }

    const float R = 6371000.0f;
    float dLat = deg2rad((float)(lat - lat0));
    float dLon = deg2rad((float)(lon - lon0));

    xOut = R * dLon * cosf(deg2rad((float)lat0));
    yOut = R * dLat;
}

float NavCore::calcDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

// ОЦЕНКА КАЧЕСТВА GPS
// Возвращает 0.0...1.0 на основе нескольких эвристик
float NavCore::estimateGpsQuality(const GpsData& gps, float jump) {
    float q = 1.0f;

    // 1. Валидность данных
    if (gps.time == 0) return 0.0f;
    if (gps.lat == 0.0 && gps.lon == 0.0) return 0.0f;

    // 2. Физическая невозможная скорость
    if (gps.speed < 0.0f || gps.speed > MAX_ROWING_SPEED * 1.5f) {
        q *= 0.05f;
    }

    // 3. Расхождение скорости GPS с оценкой IMU
    float imuSpeed = sqrtf(vx * vx + vy * vy);
    float speedDiff = fabsf(gps.speed - imuSpeed);
    if (speedDiff > GPS_SPEED_DEVIATION) {
        q *= (GPS_SPEED_DEVIATION / speedDiff);  // чем больше расхождение, тем хуже
    }

    // 4. Резкий скачок позиции (спуффинг)
    if (jump > GPS_MAX_JUMP) {
        q *= 0.05f;
    } else if (jump > GPS_MAX_JUMP * 0.5f) {
        q *= 0.5f;
    }

    // Ограничиваем диапазон
    if (q < 0.0f) q = 0.0f;
    if (q > 1.0f) q = 1.0f;

    return q;
}

// ОНЛАЙН-КАЛИБРОВКА BIAS АКСЕЛЕРОМЕТРА
// Когда лодка неподвижна, среднее ускорение = bias. Накапливаем его.
void NavCore::updateBias(const ImuData& imu, float dt) {
    float speed = sqrtf(vx * vx + vy * vy);

    // Обновляем bias только в покое и без доверия к GPS
    if (speed < MIN_SPEED_THRESHOLD && gpsQuality < 0.3f && !inStroke) {
        const float alpha = 0.01f;  // очень медленная адаптация (~1% за тик)
        axBias = axBias * (1.0f - alpha) + imu.ax * alpha;
        ayBias = ayBias * (1.0f - alpha) + imu.ay * alpha;
    }
}

// ДЕТЕКТОР ГРЕБКОВ
// Используем гистерезис: высокий порог для старта, низкий для окончания.
// Плюс проверка минимального интервала (защита от двойных срабатываний).
void NavCore::detectStroke(const ImuData& imu, uint32_t now) {
    float az = imu.az;
    uint32_t dt = now - lastStrokeTime;

    if (!inStroke) {
        // Начало гребка: вертикальный импульс + защита от повторного срабатывания
        if (az > STROKE_THRESHOLD_HIGH && dt > STROKE_MIN_INTERVAL) {
            inStroke = true;
            state.strokeCount++;
            lastStrokeTime = now;
        }
    } else {
        // Конец гребка: фаза возврата (лодка выравнивается)
        if (az < STROKE_THRESHOLD_LOW) {
            inStroke = false;
        }
        // Защита от зависания: если гревок затянулся — сбрасываем
        if (dt > STROKE_MAX_INTERVAL) {
            inStroke = false;
        }
    }
}

void NavCore::update(const GpsData& gps, const ImuData& imu) {
    uint32_t now = millis();

    // Инициализация
    if (startTime == 0) {
        startTime = now;
        lastTime = now;
        lastGpsTime = now;
    }

    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.5f)  dt = 0.5f;
    lastTime = now;

    state.totalTime = now - startTime;

    // ОПРЕДЕЛЯЕМ РЕЖИМ РАБОТЫ
    enum class Mode { EXCELLENT, POOR, DEAD };
    Mode mode;
    
    if (gpsQuality > GPS_QUALITY_EXCELLENT) {
        mode = Mode::EXCELLENT;
    } else if (gpsQuality > GPS_QUALITY_DEAD) {
        mode = Mode::POOR;
    } else {
        mode = Mode::DEAD;
    }

    // IMU PREDICTION
    float ax = imu.ax - axBias;
    float ay = imu.ay - ayBias;

    // Компенсация гравитации
    float roll = atan2f(imu.ay, imu.az);
    ax -= sinf(roll) * GRAVITY;

    // Интегрирование
    vx += ax * dt;
    vy += ay * dt;
    x  += vx * dt;
    y  += vy * dt;

    // Адаптивное затухание в зависимости от режима
    float damping;
    switch (mode) {
        case Mode::EXCELLENT: damping = DAMPING_EXCELLENT; break;
        case Mode::POOR:      damping = DAMPING_POOR;      break;
        case Mode::DEAD:      damping = DAMPING_DEAD;      break;
    }
    vx *= damping;
    vy *= damping;

    // GPS MEASUREMENT UPDATE
    bool gpsValid = (gps.time > 0);

    if (gpsValid && mode != Mode::DEAD) {
        float gpsX = 0.0f, gpsY = 0.0f;
        gpsToXY(gps.lat, gps.lon, gpsX, gpsY);

        float jump = calcDistance(x, y, gpsX, gpsY);
        float newQuality = estimateGpsQuality(gps, jump);

        // Плавное обновление качества
        gpsQuality = gpsQuality * 0.92f + newQuality * 0.08f;

        // Коррекция только если GPS достаточно хорош
        if (gpsQuality > GPS_QUALITY_POOR && jump < 100.0f) {
            float w = gpsQuality * 0.5f;

            // Позиционная коррекция
            x = x * (1.0f - w) + gpsX * w;
            y = y * (1.0f - w) + gpsY * w;

            // Коррекция скорости
            float imuSpeed = sqrtf(vx * vx + vy * vy);
            if (imuSpeed > MIN_SPEED_THRESHOLD && gps.speed > MIN_SPEED_THRESHOLD) {
                float ratio = gps.speed / imuSpeed;
                ratio = constrain(ratio, 0.5f, 2.0f);
                ratio = 1.0f + (ratio - 1.0f) * w;
                vx *= ratio;
                vy *= ratio;
            } else if (gps.speed > MIN_SPEED_THRESHOLD && imuSpeed <= MIN_SPEED_THRESHOLD) {
                float dx = gpsX - x;
                float dy = gpsY - y;
                float dist = sqrtf(dx * dx + dy * dy);
                if (dist > 0.1f) {
                    vx = (dx / dist) * gps.speed * w;
                    vy = (dy / dist) * gps.speed * w;
                }
            }
        }

        lastGpsTime = now;
    } else {
        // GPS пропал или полностью невалиден — качество деградирует
        if (mode == Mode::DEAD) {
            gpsQuality *= 0.95f;  // быстрее деградирует в DEAD режиме
        } else {
            gpsQuality *= GPS_QUALITY_DECAY;
        }
    }

    // ZUPT — Zero Velocity Update (адаптивный)
    float azDyn = fabsf(imu.az - GRAVITY);
    float totalAccel = sqrtf(ax * ax + ay * ay + azDyn * azDyn);

    // Порог ZUPT зависит от режима
    float zuptThreshold = (mode == Mode::DEAD) ? ZUPT_ACCEL_THRESHOLD_DEAD : ZUPT_ACCEL_THRESHOLD_GOOD;

    if (totalAccel < zuptThreshold && !inStroke) {
        // В режиме DEAD — жёсткое обнуление
        if (mode == Mode::DEAD) {
            vx = 0.0f;
            vy = 0.0f;
        } else {
            // В других режимах — плавное затухание
            vx *= 0.3f;
            vy *= 0.3f;
        }
    }

    // Velocity clamping — если скорость очень мала и нет GPS, обнуляем
    float speed = sqrtf(vx * vx + vy * vy);
    if (mode == Mode::DEAD && speed < VELOCITY_CLAMP_THRESHOLD) {
        vx = 0.0f;
        vy = 0.0f;
    }

    // HARD LIMIT

    speed = sqrtf(vx * vx + vy * vy);
    if (speed > MAX_ROWING_SPEED) {
        float scale = MAX_ROWING_SPEED / speed;
        vx *= scale;
        vy *= scale;
        speed = MAX_ROWING_SPEED;
    }

    // ДЕТЕКТОР ГРЕБКОВ
    detectStroke(imu, now);


    // ОНЛАЙН-КАЛИБРОВКА BIAS (адаптивная скорость)
    float imuSpeed = sqrtf(vx * vx + vy * vy);
    if (imuSpeed < MIN_SPEED_THRESHOLD && !inStroke) {
        float biasAlpha = (mode == Mode::DEAD) ? BIAS_ALPHA_FAST : BIAS_ALPHA_SLOW;
        axBias = axBias * (1.0f - biasAlpha) + imu.ax * biasAlpha;
        ayBias = ayBias * (1.0f - biasAlpha) + imu.ay * biasAlpha;
    }

    // ВЫХОДНЫЕ МЕТРИКИ
    state.distance = sqrtf(x * x + y * y);
    state.currentSpeed = speed;

    float t = state.totalTime / 1000.0f;
    state.avgSpeed = (t > 0.0f) ? (state.distance / t) : 0.0f;

    if (state.currentSpeed > MIN_SPEED_THRESHOLD) {
        state.pace500 = 500.0f / state.currentSpeed;
    } else {
        state.pace500 = 0.0f;
    }

    // Статусы
    state.gpsValid   = gpsValid;
    state.gpsTrusted = (gpsQuality > GPS_QUALITY_POOR);
    state.imuValid   = true;
    state.valid      = true;
}

const NavigationState& NavCore::get() const {
    return state;
}