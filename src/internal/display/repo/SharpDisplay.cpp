#include "internal/display/repo/SharpDisplay.h"
#include <Arduino.h>
#include <stdio.h>

SharpDisplayRepo::SharpDisplayRepo()
    : display(18, 23, 5, 400, 240)
{}

bool SharpDisplayRepo::init() {
    display.begin();
    display.clearDisplay();
    display.refresh();
    
    // Сетка координат (400x240)
    // Формат: {x, y_значения, width, height}
    
    areaUtc       = {10,  10, 120, 20};   // Верхний бар
    areaDistance  = {10,  65, 230, 50};   // Дистанция 
    areaPace      = {240, 70, 150, 45};   // Темп 
    
    areaTime      = {10,  155, 120, 35};  // Нижний ряд: Время 
    areaStrokes   = {150, 155, 100, 35};  // Нижний ряд: Гребки 
    areaSpeed     = {260, 155, 140, 35};  // Нижний ряд: Скорость 
    areaStatus    = {10,  210, 380, 25};  // Самый низ: Статус
    
    drawStaticUI();
    cached.initialized = true;
    
    return true;
}

void SharpDisplayRepo::drawStaticUI() {
    display.clearDisplay();
    
    // Верхняя разделительная линия
    display.drawLine(10, 35, 390, 35, 0);
    
    // Подписи параметров 
    display.setTextSize(2);
    display.setTextColor(0);
    
    display.setCursor(areaDistance.x, areaDistance.y - 18);
    display.print("DISTANCE");
    
    display.setCursor(areaPace.x, areaPace.y - 18);
    display.print("PACE 500m");
    
    // Разделитель перед нижним блоком
    display.drawLine(10, 130, 390, 130, 0);
    
    display.setCursor(areaTime.x, areaTime.y - 18);
    display.print("TIME");
    
    display.setCursor(areaStrokes.x, areaStrokes.y - 18);
    display.print("STROKES");
    
    display.setCursor(areaSpeed.x, areaSpeed.y - 18);
    display.print("SPEED");
    
    // Разделитель перед статусом
    display.drawLine(10, 190, 390, 190, 0);
    
    display.refresh();
}

void SharpDisplayRepo::clearArea(const Area& area) {
    display.fillRect(area.x, area.y, area.w, area.h, 1);
}


void SharpDisplayRepo::formatPace(char* buf, float paceSeconds) {
    if (paceSeconds <= 0.0f || isnan(paceSeconds)) {
        strcpy(buf, "--:--");
        return;
    }
    int m = (int)(paceSeconds / 60.0f);
    int s = (int)roundf(paceSeconds) % 60;
    sprintf(buf, "%02d:%02d", m, s);
}

void SharpDisplayRepo::formatUtcTime(char* buf, uint32_t unixTime) {
    if (unixTime == 0) {
        strcpy(buf, "--:--");
        return;
    }

    unixTime += 10800; 
    
    uint32_t totalSeconds = unixTime % 86400;
    int hours = totalSeconds / 3600;
    int minutes = (totalSeconds % 3600) / 60;
    sprintf(buf, "%02d:%02d", hours, minutes);
}

void SharpDisplayRepo::formatStopwatch(char* buf, uint32_t totalMs) {
    uint32_t totalSec = totalMs / 1000;
    int m = totalSec / 60;
    int s = totalSec % 60;
    sprintf(buf, "%02d:%02d", m, s);
}


void SharpDisplayRepo::updateDynamic(const NavigationState& state) {
    bool changed = false;
    char buf[16];

    // 1. Мировое время 
    uint32_t currentMinute = state.utcTime / 60;
    static uint32_t lastMinute = UINT32_MAX;
    if (currentMinute != lastMinute) {
        clearArea(areaUtc);
        display.setTextSize(2);
        display.setTextColor(0);
        display.setCursor(areaUtc.x, areaUtc.y);
        formatUtcTime(buf, state.utcTime);
        display.print(buf);
        lastMinute = currentMinute;
        changed = true;
    }

    // 2. Дистанция 
    if (abs(state.distance - cached.distance) > 0.1f) {
        clearArea(areaDistance);
        display.setTextSize(6);
        display.setTextColor(0);
        display.setCursor(areaDistance.x, areaDistance.y);
        display.print(state.distance, 1);
        cached.distance = state.distance;
        changed = true;
    }

    // 3. Темп 500м 
    if (abs(state.pace500 - cached.pace500) > 0.5f) {
        clearArea(areaPace);
        display.setTextSize(5);
        display.setTextColor(0);
        display.setCursor(areaPace.x, areaPace.y);
        formatPace(buf, state.pace500);
        display.print(buf);
        cached.pace500 = state.pace500;
        changed = true;
    }

    // 4. Общее время 
    uint32_t timeSec = state.totalTime / 1000;
    if (timeSec != cached.totalTime) {
        clearArea(areaTime);
        display.setTextSize(4);
        display.setTextColor(0);
        display.setCursor(areaTime.x, areaTime.y);
        formatStopwatch(buf, state.totalTime);
        display.print(buf);
        cached.totalTime = timeSec;
        changed = true;
    }

    // 5. Гребки 
    if (state.strokeCount != cached.strokeCount) {
        clearArea(areaStrokes);
        display.setTextSize(4);
        display.setTextColor(0);
        display.setCursor(areaStrokes.x, areaStrokes.y);
        display.print(state.strokeCount);
        cached.strokeCount = state.strokeCount;
        changed = true;
    }

    // 6. Текущая скорость 
    if (abs(state.currentSpeed - cached.currentSpeed) > 0.05f) {
        clearArea(areaSpeed);
        display.setTextSize(4);
        display.setTextColor(0);
        display.setCursor(areaSpeed.x, areaSpeed.y);
        display.print(state.currentSpeed, 1); 
        cached.currentSpeed = state.currentSpeed;
        changed = true;
    }

    // 7. Статус GPS 
    if (state.gpsValid != cached.gpsValid || state.gpsTrusted != cached.gpsTrusted) {
        clearArea(areaStatus);
        display.setTextSize(2);
        display.setTextColor(0);
        display.setCursor(areaStatus.x, areaStatus.y);
        
        if (state.gpsValid) {
            display.print("GPS: ");
            display.print(state.gpsTrusted ? "LOCKED" : "SEARCHING...");
        } else {
            display.print("GPS: NO SIGNAL");
        }
        cached.gpsValid = state.gpsValid;
        cached.gpsTrusted = state.gpsTrusted;
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