#include "internal/app/app.h"

App app;

void setup(void) {
	Serial.begin(115200);
    app.init();
}

void loop() {
	app.update();
}
