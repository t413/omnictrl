#pragma once
#include "dynamics_base.h"
#include <pid.h>

#define MAX_TILT 30.0

class TriOmni : public DynamicsBase {
  PIDCtrl balanceCtrl_ = PIDCtrl(0.6, 0.0, 0.1, 2); //inputs angle -> outputs torque (in A)
  PIDCtrl balanceSpeedCtrl_ = PIDCtrl(90.0, 0.0, 0.4, 30, 60); //inputs speed -> outputs angle
  PIDCtrl balanceYawCtrl_ = PIDCtrl(13.0, 0.0, 0.1, 2, 100); //outputs torque
  uint32_t lastBalanceChange_ = 0;
  bool yawCtrlEnabled_ = false;
  bool isBalancing_ = false;
  float pitchFwd_ = 0.0f;
  String status_;

public:
  virtual ~TriOmni() = default;
  TriOmni(MotionTask*);

  virtual void init();
  virtual void enable(bool);
  virtual void iterate(uint32_t now, const SharedState&, DriveManager&);
  virtual void updateState(uint32_t now, SharedState&);
  virtual void resetPids();
  virtual String getStatus() const { return status_; }
  virtual bool isBalancing() const { return isBalancing_; }
  void calcBalanceSpeeds(const SharedState& state, float& fwd, float& yaw);
};
