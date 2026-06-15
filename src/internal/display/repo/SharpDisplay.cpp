#include "internal/display/repo/SharpDisplay.h"
#include <Arduino.h>

SharpDisplayRepo::SharpDisplayRepo()
    : display(18, 23, 5, 400, 240)
{}

bool SharpDisplayRepo::init() {
    display.begin();
    display.clearDisplay();
    display.refresh();
    
    // Координаты областей
    areaDistance = {10,  50, 180, 40};  // Значение на Y=50, подпись на Y=36
    areaSpeed    = {210, 50, 180, 40};
    
    areaAvg      = {10,  105, 120, 30}; // Значение на Y=105, подпись на Y=91
    areaPace     = {140, 105, 120, 30};
    areaStroke   = {270, 105, 120, 30};
    
    areaTime     = {10,  160, 380, 30}; // Значение на Y=150, подпись на Y=136
    areaStatus   = {10,  220, 380, 30}; // Значение на Y=200
    
    drawStaticUI();
    cached.initialized = true;
    
    return true;
}

// Статические элементы 
void SharpDisplayRepo::drawStaticUI() {
    display.clearDisplay();
    
    // 1. Заголовок (размер 2 = высота ~16px, занимает Y от 5 до 21)
    display.setTextSize(2);
    display.setTextColor(0);
    display.setCursor(10, 5);
    display.print("ROWING");
    
    // 2. Разделительная линия под заголовком (Y=30, дает 9px отступа от текста)
    display.drawLine(10, 30, 390, 30, 0);
    
    // 3. Подписи для метрик (размер 1 = высота 8px)
    display.setTextSize(1);
    
    display.setCursor(areaDistance.x, areaDistance.y - 14);
    display.print("DISTANCE (m)");
    
    display.setCursor(areaSpeed.x, areaSpeed.y - 14);
    display.print("SPEED (m/s)");
    
    display.setCursor(areaAvg.x, areaAvg.y - 14);
    display.print("AVG SPD");
    
    display.setCursor(areaPace.x, areaPace.y - 14);
    display.print("PACE/500m");
    
    display.setCursor(areaStroke.x, areaStroke.y - 14);
    display.print("STROKES");
    
    display.setCursor(areaTime.x, areaTime.y - 14);
    display.print("TIME");
    
    // Линия после второй строки метрик (Avg/Pace/Strokes заканчиваются около Y=135)
    display.drawLine(10, 138, 390, 138, 0);
    
    display.refresh();
}

// Очиcтить область перед рисованием нового значения
void SharpDisplayRepo::clearArea(const Area& area) {
    display.fillRect(area.x, area.y, area.w, area.h, 1); // 1 = белый фон
}

// Обновить только изменившиеся значения
void SharpDisplayRepo::updateDynamic(const NavigationState& state) {
    bool changed = false;
    
    // Distance
    if (abs(state.distance - cached.distance) > 0.05f) {
        clearArea(areaDistance);
        display.setTextSize(3);
        display.setTextColor(0);
        display.setCursor(areaDistance.x, areaDistance.y);
        display.print(state.distance, 1);
        cached.distance = state.distance;
        changed = true;
    }
    
    // Speed
    if (abs(state.currentSpeed - cached.currentSpeed) > 0.01f) {
        clearArea(areaSpeed);
        display.setTextSize(3);
        display.setTextColor(0);
        display.setCursor(areaSpeed.x, areaSpeed.y);
        display.print(state.currentSpeed, 2);
        cached.currentSpeed = state.currentSpeed;
        changed = true;
    }
    
    // Avg Speed
    if (abs(state.avgSpeed - cached.avgSpeed) > 0.01f) {
        clearArea(areaAvg);
        display.setTextSize(2);
        display.setTextColor(0);
        display.setCursor(areaAvg.x, areaAvg.y);
        display.print(state.avgSpeed, 2);
        cached.avgSpeed = state.avgSpeed;
        changed = true;
    }
    
    // Pace
    if (abs(state.pace500 - cached.pace500) > 0.5f) {
        clearArea(areaPace);
        display.setTextSize(2);
        display.setTextColor(0);
        display.setCursor(areaPace.x, areaPace.y);
        if (state.pace500 > 0) {
            display.print(state.pace500, 0);
        } else {
            display.print("--");
        }
        cached.pace500 = state.pace500;
        changed = true;
    }
    
    // Stroke Count
    if (state.strokeCount != cached.strokeCount) {
        clearArea(areaStroke);
        display.setTextSize(2);
        display.setTextColor(0);
        display.setCursor(areaStroke.x, areaStroke.y);
        display.print(state.strokeCount);
        cached.strokeCount = state.strokeCount;
        changed = true;
    }
    
    // Time
    uint32_t timeSec = state.totalTime / 1000;
    if (timeSec != cached.totalTime) {
        clearArea(areaTime);
        display.setTextSize(2);
        display.setTextColor(0);
        display.setCursor(areaTime.x, areaTime.y);
        
        uint32_t minutes = timeSec / 60;
        uint32_t seconds = timeSec % 60;
        if (minutes < 10) display.print("0");
        display.print(minutes);
        display.print(":");
        if (seconds < 10) display.print("0");
        display.print(seconds);
        
        cached.totalTime = timeSec;
        changed = true;
    }
    
    // GPS Status
    if (state.gpsValid != cached.gpsValid) {
        clearArea(areaStatus);
        display.setTextSize(1);
        display.setTextColor(0);
        display.setCursor(areaStatus.x, areaStatus.y);
        
        if (state.gpsValid) {
            display.print("GPS: OK");
            if (state.gpsTrusted) {
                display.print(" [TRUSTED]");
            } else {
                display.print(" [WEAK]");
            }
        } else {
            display.print("GPS: NO SIGNAL");
        }
        
        cached.gpsValid = state.gpsValid;
        changed = true;
    }
    
    if (changed) {
        display.refresh();
    }
}

void SharpDisplayRepo::render(const NavigationState& state) {
    if (!cached.initialized) {
        drawStaticUI();
        cached.initialized = true;
    }
    
    updateDynamic(state);
}