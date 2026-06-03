#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

#include "internal/display/service/displayrepo.h"

class SharpDisplayRepo : public displayrepo {
private:
    Adafruit_SharpMem display;

public:
    SharpDisplayRepo();

    bool init() override;

    void render(const NavigationState& state) override;
};