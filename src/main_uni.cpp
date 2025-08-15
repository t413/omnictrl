#include "version.h"
#include <controller.h>
#include <AlfredoCRSF.h>
#include <multimotor/can/odrive.h>
#include <multimotor/can/can_esp32_twai.h>
#include <multimotor/can/can_drive_manager.h>
#include <uni_balancer.h>

CanEsp32Twai twaiInterface_;
CanDriveManager driveManager(&twaiInterface_);
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

  driveManager.addDrive(&motor);
  dynamics.balCtrl_.tuneScale *= 15.0; //scale up PIDs

  ctrl.setup(&dynamics, &driveManager, &crsf_);
  ctrl.lowVoltageCutoff_ = LOW_BATTERY_VOLTAGE;
}

void loop() {
  ctrl.loop();
}
