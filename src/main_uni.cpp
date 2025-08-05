#include "version.h"
#include <controller.h>
#include <AlfredoCRSF.h>
#include <can/odrive.h>
#include <can/can_esp32_twai.h>
#include <uni_balancer.h>

CanEsp32Twai twaiInterface_;
ODriveDriver motor(23, &twaiInterface_);

#define PIN_CRSF_RX 5
#define PIN_CRSF_TX 6
#define PIN_CAN_RX 1
#define PIN_CAN_TX 2

#define LOW_BATTERY_VOLTAGE 21.0

AlfredoCRSF crsf_;
Controller ctrl(GIT_VERSION);
UniBalancer dynamics(&ctrl);

void setup() {
  Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  crsf_.begin(Serial1);

  twaiInterface_.setup(PIN_CAN_RX, PIN_CAN_TX, 250000, &Serial);

  ctrl.addDrive(&motor);
  ctrl.setInterface(&twaiInterface_);
  dynamics.balCtrl_.tuneScale *= 15.0; //scale up PIDs

  ctrl.setup(&dynamics, &crsf_);
  ctrl.lowVoltageCutoff_ = LOW_BATTERY_VOLTAGE;
}

void loop() {
  ctrl.loop();
}
