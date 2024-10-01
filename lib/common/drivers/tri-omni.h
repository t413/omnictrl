#pragma once
#include "dynamics_driver.h"
#include "../pid.h"

class TriOmni : public DynamicsDriver {
  Controller* ctrl_ = nullptr;
  uint8_t drivesIDs_[3] = {0};
  PIDCtrl yawCtrl_ = PIDCtrl(0.28, 0.08, 0.0, 10);
  PIDCtrl balanceCtrl_ = PIDCtrl(17.0, 0.50, 0.06, 30);
  PIDCtrl balanceSpeedCtrl_ = PIDCtrl(12.6, 4.0, 0.0, 18);
  bool lastArmed_ = false;
  bool yawCtrlEnabled_ = false;
  bool isBalancing_ = false;
  float maxSpeed_ = 0.0;
  float filteredFwdSpeed_ = 0.0;
  float filteredFwdSpeedAlpha_ = 0.04;

public:
  TriOmni();
  DynamicsDriver* canInit(Controller*) override;
  void loop() override;
  Adjustables getAdjustables() override;
  bool canPrint() const;
};
