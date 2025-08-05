#include "version.h"
#include <controller.h>
#include <AlfredoCRSF.h>
#include <can/cybergear.h>
#include <can/can_esp32_twai.h>
#include <tri_omni.h>

CanEsp32Twai twaiInterface_;
CyberGearDriver mot_back(0x7D, &twaiInterface_);
CyberGearDriver mot_right(0x7E, &twaiInterface_);
CyberGearDriver mot_left(0x7F, &twaiInterface_);

#define PIN_CRSF_RX 5
#define PIN_CRSF_TX 6
#define PIN_CAN_RX 1
#define PIN_CAN_TX 2

#define LOW_BATTERY_VOLTAGE 21.0

AlfredoCRSF crsf_;
Controller ctrl(GIT_VERSION);
TriOmni triOmni(&ctrl);

void setup() {
  Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  crsf_.begin(Serial1);

  twaiInterface_.setup(PIN_CAN_RX, PIN_CAN_TX, 1000000, &Serial); //1megabaud

  ctrl.addDrive(&mot_back);
  ctrl.addDrive(&mot_right);
  ctrl.addDrive(&mot_left);
  ctrl.setInterface(&twaiInterface_);

  ctrl.setup(&triOmni);
  ctrl.lowVoltageCutoff_ = LOW_BATTERY_VOLTAGE;
}

void loop() {
  ctrl.loop();
}
