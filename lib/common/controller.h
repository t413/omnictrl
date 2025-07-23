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
  PIDCtrl yawCtrl_ = PIDCtrl(2.8, 0, 0.0, 30);
  PIDCtrl balanceCtrl_ = PIDCtrl(0.6, 0.0, 0.1, 2); //outputs torque in A
  PIDCtrl balanceSpeedCtrl_ = PIDCtrl(90.0, 0.0, 0.4, 30, 60); //outputs speed
  PIDCtrl balanceYawCtrl_ = PIDCtrl(24.0, 0.0, 0.8, 2, 100); //outputs torque
  float fwdSpeed_ = 0.0; //fwd/back speed from motor drives
  float yawSpeed_ = 0.0;
  Telem telem_;
  uint32_t lastBalanceChange_ = 0;
  bool yawCtrlEnabled_ = false;
  bool isBalancing_ = false;
  DisplayHandler display_;

  static const uint8_t NUM_ADJUSTABLES = 15;
  float* adjustables_[NUM_ADJUSTABLES] = {
      &gyroScale_,
      &balanceCtrl_.P, &balanceCtrl_.I, &balanceCtrl_.D, &balanceCtrl_.rampLimit,
      &balanceSpeedCtrl_.P, &balanceSpeedCtrl_.I, &balanceSpeedCtrl_.D, &balanceSpeedCtrl_.rampLimit,
      &balanceYawCtrl_.P, &balanceYawCtrl_.I, &balanceYawCtrl_.D,
      &yawCtrl_.P, &yawCtrl_.I, &yawCtrl_.D,
  };
  String adjNames_[NUM_ADJUSTABLES] = {
      "gyroSc",
      "b.P", "b.I", "b.D", "b.Rl",
      "b.spdP", "b.spdI", "b.spdD", "b.spdRl",
      "b.yawP", "b.yawI", "b.yawD",
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
