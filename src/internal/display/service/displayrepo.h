#pragma once

#include "internal/navcore/domain/NavigationState.h"

class displayrepo {
public:
    virtual ~displayrepo() = default;

    virtual bool init() = 0;

    virtual void render(const NavigationState& state) = 0;
};