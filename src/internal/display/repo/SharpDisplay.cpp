#include "internal/display/repo/SharpDisplay.h"

SharpDisplayRepo::SharpDisplayRepo()
    : display(
        18,   
        23,   
        5,    
        400, 
        240   
      )
{}

bool SharpDisplayRepo::init() {
    display.begin();

    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(0);

    display.refresh();

    return true;
}

void SharpDisplayRepo::render(const NavigationState& state) {
    display.clearDisplay();

    display.setCursor(10, 20);
    display.print("DIST:");
    display.print(state.distance, 1);
    display.print(" m");

    display.setCursor(10, 50);
    display.print("SPD:");
    display.print(state.currentSpeed, 2);
    display.print(" m/s");

    display.setCursor(10, 80);
    display.print("AVG:");
    display.print(state.avgSpeed, 2);

    display.setCursor(10, 110);
    display.print("PACE:");
    display.print(state.pace500, 0);

    display.setCursor(10, 140);
    display.print("STR:");
    display.print(state.strokeCount);

    display.setCursor(10, 170);
    display.print("TIME:");
    display.print(state.totalTime / 1000);

    display.refresh();
}

