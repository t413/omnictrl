#include "version.h"
#include <controller.h>

Controller ctrl(GIT_VERSION);

void setup() {
  ctrl.setup();
}
void loop() {
  ctrl.loop();
}
