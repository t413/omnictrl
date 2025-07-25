#pragma once

#include "pid.h"
#include "rfprotocol.h"
#include "displayhandler.h"
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
  MotionControl lastEspNowCmd_;
  MotionControl lastCrsfCmd_;
  MotionControl* activeTx_ = nullptr; //who's in control
  uint8_t remoteMac_[6] = {0}; //14:2B:2F:B0:52:A0 is tx
  bool enabled_ = false;
  Madgwick imuFilt_;
  float gyroScale_ = 1.0;
  float gyroZ = 0;
  PIDCtrl yawCtrl_ = PIDCtrl(0.28, 0.08, 0.0, 10);
  PIDCtrl balanceCtrl_ = PIDCtrl(0.06, 0.0, 0.02, 2); //outputs torque in A
  PIDCtrl balanceSpeedCtrl_ = PIDCtrl(9.0, 0.0, 0.04, 20, 55); //outputs speed
  PIDCtrl balanceYawCtrl_ = PIDCtrl(2.4, 0.0, 0.08, 1, 100); //outputs torque
  float fwdSpeed_ = 0.0; //fwd/back speed from motor drives
  float yawSpeed_ = 0.0;
  Telem telem_;
  uint32_t lastBalanceChange_ = 0;
  bool yawCtrlEnabled_ = false;
  bool isBalancing_ = false;
  DisplayHandler display_;

  static const uint8_t NUM_ADJUSTABLES = 12;
  float* adjustables_[NUM_ADJUSTABLES] = {
      &gyroScale_,
      &balanceCtrl_.P, &balanceCtrl_.I, &balanceCtrl_.D, &balanceCtrl_.rampLimit,
      &balanceSpeedCtrl_.P, &balanceSpeedCtrl_.I, &balanceSpeedCtrl_.D, &balanceSpeedCtrl_.rampLimit,
      &balanceYawCtrl_.P, &balanceYawCtrl_.I, &balanceYawCtrl_.D,
  };
  String adjNames_[NUM_ADJUSTABLES] = {
      "gyroSc",
      "balP", "balI", "balD", "balRl",
      "spdP", "spdI", "spdD", "spdRl",
      "yawP", "yawI", "yawD",
  };
  uint8_t selectedTune_ = NUM_ADJUSTABLES; //none selected

public:
  Controller(String version);
  ~Controller();

  void setup();
  void loop();

  bool isLinkUp(uint32_t) const;
  MotionControl getCrsfCtrl(uint32_t now) const;
  uint8_t getValidDriveCount() const;
  void resetPids();

  void handleRxPacket(const uint8_t* mac, const uint8_t* buf, uint8_t len);
  void drawLCD(const uint32_t);
  bool updateIMU();
  const String version_;
};
