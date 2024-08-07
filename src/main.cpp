#include <controller.h>
#include "version.h"

Controller ctrl(GIT_VERSION);

void setup() {
  ctrl.setup();
}
void loop() {
  ctrl.loop();
}
