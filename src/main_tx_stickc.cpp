#include "version.h"
#include <rcremote.h>
RCRemote ctrl(GIT_VERSION);

void setup() {
  ctrl.setup();
}
void loop() {
  ctrl.loop();
}
