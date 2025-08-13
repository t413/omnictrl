#include "tri_omni.h"
#include <multimotor/motordrive.h>
#include <AlfredoCRSF.h>
#include <MadgwickAHRS.h>
#include <controller.h>
#include <utils.h>

TriOmni::TriOmni(Controller* ctrl) : DynamicsBase(ctrl) { }

void TriOmni::init() {
  ctrl_->addAdjustable(&balanceCtrl_.P, "b.P");
  ctrl_->addAdjustable(&balanceCtrl_.I, "b.I");
  ctrl_->addAdjustable(&balanceCtrl_.D, "b.D");
  ctrl_->addAdjustable(&balanceCtrl_.rampLimit, "b.Rl");
  ctrl_->addAdjustable(&balanceSpeedCtrl_.P, "spd.P");
  ctrl_->addAdjustable(&balanceSpeedCtrl_.I, "spd.I");
  ctrl_->addAdjustable(&balanceSpeedCtrl_.D, "spd.D");
  ctrl_->addAdjustable(&balanceSpeedCtrl_.rampLimit, "spd.Rl");
  ctrl_->addAdjustable(&balanceYawCtrl_.P, "b.yaw.P");
  ctrl_->addAdjustable(&balanceYawCtrl_.I, "b.yaw.I");
  ctrl_->addAdjustable(&balanceYawCtrl_.D, "b.yaw.D");
  ctrl_->addAdjustable(&yawCtrl_.P, "yaw.P");
  ctrl_->addAdjustable(&yawCtrl_.I, "yaw.I");
  ctrl_->addAdjustable(&yawCtrl_.D, "yaw.D");

  auto drives = ctrl_->getDrives();
  for (int i = 0; i < MAX_DRIVES; i++) {
    if (drives[i])
      drives[i]->setMode(MotorMode::Disabled); // start with disabled mode
  }
}

void TriOmni::enable(bool en) {
  resetPids();
  auto drives = ctrl_->getDrives();
  for (int i = 0; i < MAX_DRIVES; i++)
    if (drives[i])
      drives[i]->setMode(en? MotorMode::Speed : MotorMode::Disabled);
}

void TriOmni::iterate(uint32_t now) {
  auto drives = ctrl_->getDrives();
  auto imu = ctrl_->getImuFilter();
  auto motion = ctrl_->getActiveTx();
  auto enabled = ctrl_->getEnabled();
  auto telem = ctrl_->getTelem();

  float q[4] = {0};
  imu->getQuaternion(q);

  float R[3][3] = {0};
  Controller::quaternionToRotationMatrix(q, R);

  float pitchFwd = (atan2(-R[2][0], R[2][2]) + PI / 2) * 180.0 / PI;
  bool isUpOnEnd = abs(pitchFwd) < MAX_TILT; //more tilt allowed when balancing
  bool newbalance = isBalancing_ || isUpOnEnd;
  bool balanceModeSpeed = false;

  if (ctrl_->isCrsfActive()) { //crsf control has extra features
    auto crsf = ctrl_->getCrsf();
    bool balanceModeEn = crsf->getChannel(6) > 1400;
    balanceModeSpeed = crsf->getChannel(6) > 1600;
    newbalance &= balanceModeEn;
    yawCtrlEnabled_ = crsf->getChannel(9) > 1400;
  } else if (motion) {
    motion->maxSpeed = min(18.0f, motion->maxSpeed); //limit
  }

  if (!enabled) {
    yawCtrlEnabled_ = false;
    newbalance = false;
  } else if (isBalancing_ && newbalance && !isUpOnEnd) {
    if ((now - lastBalanceChange_) > 2000) //extra leeway when getting going
      newbalance = false;
    else if (abs(pitchFwd) > MAX_TILT * 1.2)
      newbalance = false; //way too much tilt
  }
  if (isBalancing_ != newbalance) {
    isBalancing_ = newbalance;
    Serial.printf("Balancing mode %s\n", isBalancing_? "enabled" : "disabled");
    resetPids();
    lastBalanceChange_ = now;
  }

  //main control loop
  if (ctrl_->getDriveCount()) { //at least one

    MotionControl m;
    if (motion) { m = *motion; }
    m.fwd  *= m.maxSpeed;
    m.side *= m.maxSpeed;
    m.yaw  *= m.maxSpeed;

    yawCtrl_.limit = m.maxSpeed / 4;
    float y = yawCtrlEnabled_? yawCtrl_.update(now, (-m.yaw * 100) - ctrl_->gyroZ) : -m.yaw; //convert yaw to angular rate

    #define BACK 0
    #define RGHT 1
    #define LEFT 2

    fwdSpeed_ = yawSpeed_ = 0;
    for (int i = 0; i < MAX_DRIVES; i++) {
      if (!drives[i]) continue;
      float v = drives[i]->getMotorState().velocity;
      if (i != BACK) {
        fwdSpeed_ += v * (i == RGHT? 1 : -1); //left is positive, right is negative
        yawSpeed_ += v;
      }
    }
    fwdSpeed_ /= 2;
    yawSpeed_ /= 2;

    if (lastBalanceChange_ == now) {
      for (int d = 0; d < MAX_DRIVES; d++)
        if (d != BACK && drives[d]) //back stays in speed mode
          drives[d]->setMode(isBalancing_? MotorMode::Current : MotorMode::Speed);
    }

    if (isBalancing_) {
      // Blend between angle-control and odometry speed control
      const uint32_t balanceChangeDuration = 2000; //ms
      float pitchGoal = balanceModeSpeed? balanceSpeedCtrl_.update(now, fwdSpeed_ - m.fwd) : -m.fwd;
      if (balanceModeSpeed && (now - lastBalanceChange_) < balanceChangeDuration) {
        const float blendT = (now - lastBalanceChange_) / (float)balanceChangeDuration;
        pitchGoal = blend( -m.fwd, pitchGoal, blendT);
        if (blendT < 0.5f)
          balanceSpeedCtrl_.reset(); //prevent rampup while not in control
      }
      float torqueCmd = balanceCtrl_.update(now, pitchGoal - pitchFwd); //input is angle
      torqueCmd *= 10.0; //roughly scale to Nm
      Serial.printf("(fwd %06.2f)-> [-pgoal %06.2f -p %06.2f] -> torque %06.2f\n", m.fwd, pitchGoal, pitchFwd, torqueCmd);
      m.fwd = torqueCmd; // Use torque output directly

      y = balanceYawCtrl_.update(now, (y + -m.side) - yawSpeed_); //input is speed, output is torque
      m.side = 0; //disable side
    } else {
      balanceSpeedCtrl_.reset();
      balanceCtrl_.reset();
    }
    if (enabled) {
      drives[BACK]->setSetpoint(MotorMode::Speed, isBalancing_? yawSpeed_ : (y  +   0   + m.side));
      drives[LEFT]->setSetpoint(isBalancing_? MotorMode::Current : MotorMode::Speed, y  - m.fwd   - m.side * 1.33/2);
      drives[RGHT]->setSetpoint(isBalancing_? MotorMode::Current : MotorMode::Speed, y  + m.fwd   - m.side * 1.33/2);
    }

    telem->pitch = pitchFwd; //save for next loop

  } //drive count check

  status_ = isBalancing_? "woah." : enabled? "wee!" : ":|";

  auto display = ctrl_->getDisplay();
  uint8_t rot = (isBalancing_ || (abs(pitchFwd) < MAX_TILT)) ? 0 : 2;
  display->setRotation(rot);
}

void TriOmni::resetPids() {
  yawCtrl_.reset();
  balanceCtrl_.reset();
  balanceSpeedCtrl_.reset();
  balanceYawCtrl_.reset();
}
