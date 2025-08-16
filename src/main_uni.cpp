#include "version.h"
#include <controller.h>
#include <M5Unified.h>
#include <displayhandler.h>
#include <AlfredoCRSF.h>
#include <multimotor/can/odrive.h>
#include <multimotor/can/can_esp32_twai.h>
#include <multimotor/can/can_drive_manager.h>
#include <uni_balancer.h>
#include <ESP32_DSHOT.h>

// CanEsp32Twai twaiInterface_;
// CanDriveManager driveManager(&twaiInterface_);
// ODriveDriver motor(23, &twaiInterface_);

DSHOT dshot3b;

// #define PIN_CRSF_RX 5
// #define PIN_CRSF_TX 6
// #define PIN_CAN_RX 1
// #define PIN_CAN_TX 2

// #define LOW_BATTERY_VOLTAGE 21.0

// AlfredoCRSF crsf_;
// Controller ctrl(GIT_VERSION);
// UniBalancer dynamics(&ctrl);
DisplayHandler display_;

void draw(uint32_t now, String title, String subtitle = "") {
  Serial.printf("draw title=%s subtitle=%s\n", title.c_str(), subtitle.c_str());
  display_.startFrame();
  auto bgRainbow = display_.timeRainbow(now);
  auto fg = BLACK;
  auto pageBG = BLACK;

  display_.setFont(&FreeSansBold12pt7b);
  display_.drawBorder(bgRainbow);
  display_.drawTitle(title.c_str(), fg, bgRainbow);
  display_.clearContent(pageBG, now);
  display_.drawCentered(subtitle.c_str(), pageBG);
  display_.endFrame();
}

void setup() {
  // Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  // crsf_.begin(Serial1);

  // twaiInterface_.setup(PIN_CAN_RX, PIN_CAN_TX, 250000, &Serial);

  // driveManager.addDrive(&motor);
  // dynamics.balCtrl_.tuneScale *= 15.0; //scale up PIDs

  // ctrl.setup(&dynamics, &driveManager, &crsf_);
  // ctrl.lowVoltageCutoff_ = LOW_BATTERY_VOLTAGE;

  Serial.begin(115200);
  M5.begin();
  //check if has lcd
  display_.setLCD(&M5.Lcd);

  display_.setFont(&FreeSansBold12pt7b);
  draw(millis(), "startup...");
  M5.update();
  bool doBid = M5.BtnA.isPressed();

  dshot3b.begin(38, doBid? DSHOT::DSHOT300_BIDIR : DSHOT::DSHOT300);

  dshot3b.arm();
}


int userThrottle = 0;
float filteredThr = 0.0;
float filterAlpha = 0.01;
uint32_t lastReport = 200;

void loop() {
  uint32_t now = millis();
  M5.update();
  if (M5.BtnA.wasSingleClicked()) {
    userThrottle += (userThrottle == 0)? 50 : 10; //initial bump
    if (userThrottle > 1000) { userThrottle = 0; }
    Serial.printf("BTN throttle=%d\n", userThrottle);
    lastReport = 0; //force update
  } else if (M5.BtnA.wasDoubleClicked()) {
    userThrottle = 0;
    filteredThr = 0.0;
    dshot3b.cmd(0); //stop motors
    lastReport = 0; //force update
  }

  filteredThr = (1.0 - filterAlpha) * filteredThr + filterAlpha * userThrottle;
  bool doReport = (now - lastReport) > 200;
  dshot3b.set((int) filteredThr, doReport);

  if (doReport) {
    Serial.printf(" erpm_us=%d ", dshot3b.erpm_us);
    Serial.printf(" temp=%dC ", dshot3b.telem[1]);
    Serial.printf(" bidir_cnt=%d bidir_ok=%d", dshot3b.telem_cnt, dshot3b.telem_ok_cnt);
    Serial.println();

    draw(now, "t: " + String(filteredThr), "rpm:" + String(dshot3b.erpm_us));
    lastReport = now;
  }
  delay(1);
}
