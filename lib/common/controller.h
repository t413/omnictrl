#pragma once

#include "pid.h"
#include <WString.h>
#include <AlfredoCRSF.h>
#include <I2C_MPU6886.h>
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
  I2C_MPU6886 imu_;
  Madgwick imuFilt_;
  float gyroZ = 0;
  PIDCtrl yawCtrl_ = PIDCtrl(0.28, 0.08, 0.0, 3);
  float balancePoint_ = 0.0;
  PIDCtrl balanceCtrl_ = PIDCtrl(0.28, 0.08, 0.0, 3);
  uint8_t selectedTune_ = 3;
  float maxSpeed_ = 0.0;
  bool yawCtrlEnabled_ = false;
  bool balanceModeEnabled_ = false;

public:
  Controller(String version);
  ~Controller();

  void setup();
  void loop();

  bool updateIMU();
  const String version_;
};
