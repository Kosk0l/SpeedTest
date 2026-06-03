#pragma once
#include <memory>
#include "displayrepo.h"

class display {
private:
    std::unique_ptr<displayrepo> driver;

public:
    display(std::unique_ptr<displayrepo> d) : driver(std::move(d)) {}

    bool init();
    void update(const NavigationState& state);
};