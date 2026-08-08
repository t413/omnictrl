#include "version.h"
#include <controller.h>
#include <motiontask.h>
#include <tri_omni.h>
#include <esc.h>
#include <AlfredoCRSF.h>

HardwareSerial* crsfserial = &Serial1;
HardwareSerial* telemSerial = &Serial2;

constexpr auto PIN_CRSF_RX  = GPIO_NUM_2; //grove port
constexpr auto PIN_CRSF_TX  = GPIO_NUM_1;
AlfredoCRSF crsf_;

ESCManager mgr;
ESC mot_rght(GPIO_NUM_5, &mgr, "right");
ESC mot_left(GPIO_NUM_6, &mgr, "left");
ESC mot_back(GPIO_NUM_7, &mgr, "back");
constexpr auto PIN_TELEM_TX  = GPIO_NUM_8;

#define LOW_BATTERY_VOLTAGE 6.4

Controller ctrl(GIT_VERSION);
MotionTask motion;
TriOmni triOmni(&motion, &mot_rght, &mot_left, &mot_back);

void setup() {
  D_LOG("starting up");
  crsfserial->begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  telemSerial->begin(1000000, SERIAL_8N1, 39, PIN_TELEM_TX);
  telemSerial->printf("hello debug");
  crsf_.begin(*crsfserial);
  ctrl.setup(&motion.getState(), &crsf_);
  ctrl.lowVoltageCutoff_ = 0.0f;
  // ctrl.lowVoltageCutoff_ = LOW_BATTERY_VOLTAGE;
  mgr.setup();
  motion.setup(&triOmni, &mgr, &ctrl);
}

void loop() {
  ctrl.loop();
}
