#pragma once
#include "dynamics_base.h"
#include <pid.h>

#define MAX_TILT 30.0

class UniBalancer : public DynamicsBase {
public:
  PIDCtrl balCtrl_ = PIDCtrl(0.6, 0.0, 0.1, 2); //outputs torque in A
  PIDCtrl speedCtrl_ = PIDCtrl(90.0, 0.0, 0.4, 30, 60); //outputs speed
  float fwdSpeed_ = 0.0; //fwd/back speed from motor drives
  String status_ = "init";

  virtual ~UniBalancer() = default;
  UniBalancer(Controller*);
  MotorDrive* getMotor();

  virtual void init();
  virtual void enable(bool);
  virtual void iterate(uint32_t now);
  virtual void resetPids();
  virtual String getStatus() const { return status_; }
};
