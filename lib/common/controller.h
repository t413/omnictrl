#pragma once

#include "pid.h"
#include <WString.h>
#include <AlfredoCRSF.h>
#include <MadgwickAHRS.h>
#include <xiaomi_cybergear_driver.h>

namespace lgfx { inline namespace v1 { class LGFX_Device; } }
#ifndef NUM_DRIVES
#error "NUM_DRIVES must be defined in ini"
#endif

struct DriveCtx {
  uint8_t id_ = 0;
  XiaomiCyberGearDriver driver_;
  uint8_t lastFaults_ = 0;
  uint32_t lastStatus_ = 0;
  bool enabled_ = false;
  void requestStatus();
  void enable(bool enable);
  void setSpeed(float speed);
  bool handle(const twai_message_t& message);
  XiaomiCyberGearStatus getStatus() const;
};

class Controller {
  DriveCtx drives_[NUM_DRIVES];

  AlfredoCRSF crsf_;
  int8_t lastState_ = -1;
  bool lastLinkUp_ = false;
  Madgwick imuFilt_;
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

  static const uint8_t NUM_ADJUSTABLES = 4;
  float* adjustables_[NUM_ADJUSTABLES] = { &filteredFwdSpeedAlpha_, &balanceSpeedCtrl_.P, &balanceSpeedCtrl_.I, &balanceSpeedCtrl_.D};
  String adjNames_[NUM_ADJUSTABLES] = { "a", "bP", "bI", "bD" };
  uint8_t selectedTune_ = NUM_ADJUSTABLES; //none selected

public:
  Controller(String version);
  ~Controller();

  void setup();
  void loop();

  uint8_t getValidDriveCount() const;

  void drawLCD(const uint32_t);
  bool updateIMU();
  const String version_;
};

void drawCentered(const char* text, lgfx::v1::LGFX_Device*, uint16_t bg);
