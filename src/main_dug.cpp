#include "version.h"
#include <controller.h>
#include <motiontask.h>
#include <AlfredoCRSF.h>
#include <multimotor/can/cybergear.h>
#include <multimotor/can/can_esp32_twai.h>
#include <multimotor/can/can_drive_manager.h>
#include <tri_omni.h>

CanEsp32Twai twaiInterface_;
CanDriveManager driveManager(&twaiInterface_);
CyberGearDriver mot_back(0x7D, &twaiInterface_);
CyberGearDriver mot_right(0x7E, &twaiInterface_);
CyberGearDriver mot_left(0x7F, &twaiInterface_);

constexpr int PIN_CRSF_RX  = 5;
constexpr int PIN_CRSF_TX  = 6;
//PIN_LEDS is set in .ini
constexpr int PIN_CAN_RX  = 1;
constexpr int PIN_CAN_TX  = 2;

#define LOW_BATTERY_VOLTAGE 21.0

AlfredoCRSF crsf_;
Controller ctrl(GIT_VERSION);
MotionTask motion;
TriOmni triOmni(&motion);

void setup() {
  Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  crsf_.begin(Serial1);

  twaiInterface_.setup(PIN_CAN_RX, PIN_CAN_TX, 1000000, &Serial); //1megabaud

  driveManager.addDrive(&mot_back);
  driveManager.addDrive(&mot_right);
  driveManager.addDrive(&mot_left);

  ctrl.setup(&motion.getState(), &crsf_);
  ctrl.lowVoltageCutoff_ = LOW_BATTERY_VOLTAGE;
  motion.setup(&triOmni, &driveManager, &ctrl);
}

void loop() {
  ctrl.loop();
}
