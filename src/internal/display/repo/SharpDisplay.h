#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include "internal/display/service/displayrepo.h"
#include "internal/navcore/domain/NavigationState.h"

class SharpDisplayRepo : public displayrepo {
private:
    Adafruit_SharpMem display;

    struct CachedState {
        uint32_t utcTime = 0;
        float distance = -1.0f;
        float pace500 = -1.0f;
        uint32_t totalTime = UINT32_MAX;
        uint32_t strokeCount = UINT32_MAX;
        float currentSpeed = -1.0f;
        bool gpsValid = false;
        bool initialized = false;
        bool gpsTrusted = false;
    } cached;

    // Координаты областей для частичного обновления
    struct Area {
        int16_t x, y, w, h;
    };

    Area areaUtc;
    Area areaDistance;
    Area areaPace;
    Area areaTime;
    Area areaStrokes;
    Area areaSpeed;
    Area areaStatus;

    void drawStaticUI();
    void updateDynamic(const NavigationState& state);
    void clearArea(const Area& area);
    
    // Хелперы для форматирования
    void formatPace(char* buf, float paceSeconds);
    void formatUtcTime(char* buf, uint32_t unixTime);
    void formatStopwatch(char* buf, uint32_t totalMs);

public:
    SharpDisplayRepo();
    bool init() override;
    void render(const NavigationState& state) override;
};