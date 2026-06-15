#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

#include "internal/display/service/displayrepo.h"
#include "internal/navcore/domain/NavigationState.h"

class SharpDisplayRepo : public displayrepo {
private:
    Adafruit_SharpMem display;

    // Кэш предыдущих значений для минимизации обновлений
    struct CachedState {
        float distance = -1.0f;
        float currentSpeed = -1.0f;
        float avgSpeed = -1.0f;
        float pace500 = -1.0f;
        uint32_t strokeCount = UINT32_MAX;
        uint32_t totalTime = UINT32_MAX;
        bool gpsValid = false;
        bool initialized = false;
    } cached;

    // Координаты областей для частичного обновления
    struct Area {
        int16_t x, y, w, h;
    };

    Area areaDistance;
    Area areaSpeed;
    Area areaAvg;
    Area areaPace;
    Area areaStroke;
    Area areaTime;
    Area areaStatus;

    void drawStaticUI();
    void updateDynamic(const NavigationState& state);
    void clearArea(const Area& area);

public:
    SharpDisplayRepo();

    bool init() override;
    void render(const NavigationState& state) override;
};