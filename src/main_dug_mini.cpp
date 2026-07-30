#include "version.h"
#include <controller.h>
#include <motiontask.h>
#include <multimotor/serial/serial_drive_manager.h>
#include <multimotor/serial/lx_servo.h>
#include <tri_omni.h>

HardwareSerial* busSerial = &Serial1;
SerialDriveManager driveManager;
LXServo mot_back (0x02, &driveManager, "back");
LXServo mot_right(0x03, &driveManager, "right");
LXServo mot_left (0x04, &driveManager, "left");

#define LOW_BATTERY_VOLTAGE 6.4

Controller ctrl(GIT_VERSION);
MotionTask motion;
TriOmni triOmni(&motion, &mot_right, &mot_left, &mot_back);

void setup() {
  ctrl.setup(&motion.getState(), nullptr);
  ctrl.lowVoltageCutoff_ = 0.0f;
  // ctrl.lowVoltageCutoff_ = LOW_BATTERY_VOLTAGE;

  D_LOG("setting up drivemgr");
  // driveManager.beginSinglePin(busSerial, 1);
  driveManager.beginDualPins(busSerial, 2, 1);

  motion.setup(&triOmni, &driveManager, &ctrl);
}

void loop() {
  ctrl.loop();
}
