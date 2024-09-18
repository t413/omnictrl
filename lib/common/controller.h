#pragma once

#include "pid.h"
#include <WString.h>
#include <AlfredoCRSF.h>
#include <MadgwickAHRS.h>
#include <xiaomi_cybergear_driver.h>

#define NUM_DRIVES 3

struct DriveCtx {
  uint8_t id_ = 0;
  XiaomiCyberGearDriver driver_;
  uint8_t lastFaults_ = 0;
  uint32_t lastStatus_ = 0;
  bool enabled_ = false;
  void requestStatus();
  void enable(bool enable);
  void setSpeed(float speed);
  bool handle(twai_message_t& message);
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
  PIDCtrl balanceCtrl_ = PIDCtrl(17.0, 0.50, 0.06, 20);
  uint8_t selectedTune_ = 3;
  float maxSpeed_ = 0.0;
  bool yawCtrlEnabled_ = false;
  bool isBalancing_ = false;
  bool redrawLCD_ = false;

public:
  Controller(String version);
  ~Controller();

  void setup();
  void loop();

  bool updateIMU();
  const String version_;
};
