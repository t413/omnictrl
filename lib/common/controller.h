#pragma once

#include "pid.h"
#include "rfprotocol.h"
#include <WString.h>
#include <AlfredoCRSF.h>
#include <MadgwickAHRS.h>
#include <motordrive.h>

namespace lgfx { inline namespace v1 { class LGFX_Device; } }
#ifndef NUM_DRIVES
#warning "NUM_DRIVES must be defined in ini"
#define NUM_DRIVES 1
#endif

class Controller {
  MotorDrive* drives_[NUM_DRIVES];

  AlfredoCRSF crsf_;
  MotionControl lastMotionCmd_;
  int8_t lastState_ = -1;
  bool lastLinkUp_ = false;
  Madgwick imuFilt_;
  float gyroScale_ = 1.0;
  float gyroZ = 0;
  PIDCtrl yawCtrl_ = PIDCtrl(0.28, 0.08, 0.0, 10);
  PIDCtrl balanceCtrl_ = PIDCtrl(17.0, 0.50, 0.06, 30);
  PIDCtrl balanceSpeedCtrl_ = PIDCtrl(12.6, 4.0, 0.0, 18);
  float filteredFwdSpeed_ = 0.0;
  float filteredFwdSpeedAlpha_ = 0.04;
  float maxSpeed_ = 0.0;
  bool yawCtrlEnabled_ = false;
  bool isBalancing_ = false;
  bool redrawLCD_ = false;
  lgfx::v1::LGFX_Device* lcd_ = nullptr;

  static const uint8_t NUM_ADJUSTABLES = 10;
  float* adjustables_[NUM_ADJUSTABLES] = {
      &gyroScale_,
      &balanceCtrl_.P, &balanceCtrl_.I, &balanceCtrl_.D, &balanceCtrl_.limit,
      &filteredFwdSpeedAlpha_, &balanceSpeedCtrl_.P, &balanceSpeedCtrl_.I, &balanceSpeedCtrl_.D, &balanceSpeedCtrl_.limit,
  };
  String adjNames_[NUM_ADJUSTABLES] = {
      "gyroSc",
      "balP", "balI", "balD", "balL",
      "spdFilt", "spdP", "spdI", "spdD", "spdL"
  };
  uint8_t selectedTune_ = NUM_ADJUSTABLES; //none selected

public:
  Controller(String version);
  ~Controller();

  void setup();
  void loop();

  bool isLinkUp(uint32_t) const;
  uint8_t getValidDriveCount() const;

  void handleRxPacket(const uint8_t* buf, uint8_t len);
  void drawLCD(const uint32_t);
  bool updateIMU();
  const String version_;
};

void drawCentered(const char* text, lgfx::v1::LGFX_Device*, uint16_t bg, uint16_t lr_padding = 2);
