#include "version.h"

#ifdef IS_RC_CONTROLLER
#include <rcremote.h>
RCRemote ctrl(GIT_VERSION);
#else
#include <controller.h>
Controller ctrl(GIT_VERSION);
#endif

void setup() {
  ctrl.setup();
}
void loop() {
  ctrl.loop();
}