#include "internal/imu/domain/ImuRawData.h"
#pragma once

class imurepo {
public:
    virtual ~imurepo() = default;
    virtual bool init() = 0;

    // Получить сырые данные
    virtual bool readRaw(ImuRawData& out) = 0;
};