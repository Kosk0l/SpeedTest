#include "display.h"

bool display::init() {
    return driver->init();
}

void display::update(const NavigationState& state) {
    driver->render(state);
}
