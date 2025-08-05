#pragma once
#include "dynamics_base.h"
#include <pid.h>

#define MAX_TILT 30.0

class TriOmni : public DynamicsBase {
  PIDCtrl yawCtrl_ = PIDCtrl(2.8, 0, 0.0, 30);
  PIDCtrl balanceCtrl_ = PIDCtrl(0.6, 0.0, 0.1, 2); //outputs torque in A
  PIDCtrl balanceSpeedCtrl_ = PIDCtrl(90.0, 0.0, 0.4, 30, 60); //outputs speed
  PIDCtrl balanceYawCtrl_ = PIDCtrl(24.0, 0.0, 0.8, 2, 100); //outputs torque
  float fwdSpeed_ = 0.0; //fwd/back speed from motor drives
  float yawSpeed_ = 0.0;
  uint32_t lastBalanceChange_ = 0;
  bool yawCtrlEnabled_ = false;
  bool isBalancing_ = false;
  String status_;

public:
  virtual ~TriOmni() = default;
  TriOmni(Controller*);

  virtual void init();
  virtual void enable(bool);
  virtual void iterate(uint32_t now);
  virtual void resetPids();
  virtual String getStatus() const { return status_; }
};
