#include "version.h"
#include <controller.h>
#include <AlfredoCRSF.h>
#include <multimotor/serial/serial_drive_manager.h>
#include <multimotor/serial/lx_servo.h>
#include <tri_omni.h>

const int txRxPin = 1;
HardwareSerial* busSerial = &Serial1;
SerialDriveManager driveManager(busSerial);
LXServo mot_back(0x01, &driveManager);
LXServo mot_right(0x02, &driveManager);
LXServo mot_left(0x03, &driveManager);

#define LOW_BATTERY_VOLTAGE 21.0

Controller ctrl(GIT_VERSION);
TriOmni triOmni(&ctrl);

void setup() {

  #ifdef ARDUINO_ARCH_ESP32
  busSerial->begin(115200, SERIAL_8N1, txRxPin, txRxPin);
  pinMode(txRxPin, OUTPUT|PULLUP);
  #endif

  ctrl.setup(&triOmni, &driveManager, nullptr);
  ctrl.lowVoltageCutoff_ = LOW_BATTERY_VOLTAGE;
}

void loop() {
  ctrl.loop();
}
