#include "tri_omni.h"
#include <log.h>
#include <multimotor/motordrive.h>
#include <AlfredoCRSF.h>
#include <MadgwickAHRS.h>
#include <motiontask.h>
#include <utils.h>
#include <multimotor/drive_manager.h>

TriOmni::TriOmni(MotionTask* ctrl, MotorDrive* right, MotorDrive* left, MotorDrive* back) :
    DynamicsBase(ctrl), right_(right), left_(left), back_(back) { }

void TriOmni::init() {
  ctx_->addAdjustable(&balanceCtrl_.P, "b.P");
  ctx_->addAdjustable(&balanceCtrl_.I, "b.I");
  ctx_->addAdjustable(&balanceCtrl_.D, "b.D");
  ctx_->addAdjustable(&balanceCtrl_.rampLimit, "b.Rl");
  ctx_->addAdjustable(&balanceSpeedCtrl_.P, "spd.P");
  ctx_->addAdjustable(&balanceSpeedCtrl_.I, "spd.I");
  ctx_->addAdjustable(&balanceSpeedCtrl_.D, "spd.D");
  ctx_->addAdjustable(&balanceSpeedCtrl_.rampLimit, "spd.Rl");
  ctx_->addAdjustable(&balanceSpeedCtrl_.limit, "spd.lm");
  ctx_->addAdjustable(&balanceYawCtrl_.P, "b.yaw.P");
  ctx_->addAdjustable(&balanceYawCtrl_.I, "b.yaw.I");
  ctx_->addAdjustable(&balanceYawCtrl_.D, "b.yaw.D");
  ctx_->addAdjustable(&balanceYawCtrl_.rampLimit, "b.yaw.Rl");
  ctx_->addAdjustable(&balanceYawCtrl_.limit, "b.yaw.lm");
}

void TriOmni::enable(bool en) {
  resetPids();
  for (auto d : {right_, left_, back_})
    if (d) d->setMode(en? MotorMode::Speed : MotorMode::Disabled);
}

void TriOmni::calcBalanceSpeeds(const SharedState& state, float& fwd, float& yaw) {
  if (!right_ || !left_) return;
  float vr = right_->getMotorState().velocity;
  float vl = left_->getMotorState().velocity;
  fwd = (vr - vl) / 2.0f; // right minus left, averaged
  yaw = (vr + vl) / 2.0f; // average rotation contribution
}

void TriOmni::iterate(uint32_t now, const SharedState& state, DriveManager&) {
  auto m = state.activeCmd.m;
  bool enabled = state.getEnabled();

  float R[3][3] = {0};
  MotionTask::quaternionToRotationMatrix(state.q, R);

  float pitchFwd = (atan2(-R[2][0], R[2][2]) + PI / 2) * 180.0 / PI;
  pitchFwd_ = pitchFwd;
  bool isUpOnEnd = abs(pitchFwd) < MAX_TILT; //more tilt allowed when balancing
  bool newbalance = isBalancing_ || isUpOnEnd;
  bool balanceModeSpeed = true;

  if (state.crsfActive) { //crsf control has extra features
    bool balanceModeEn = state.crsfChans[6] > 1400;
    balanceModeSpeed = state.crsfChans[6] > 1600;
    newbalance &= balanceModeEn;
  } else {
    m.maxSpeed = constrain(m.maxSpeed, 8.0f, 20.0f); //limit
  }

  if (!enabled) {
    newbalance = false;
  } else if (isBalancing_ && newbalance && !isUpOnEnd) {
    if ((now - lastBalanceChange_) > 2000) //extra leeway when getting going
      newbalance = false;
    else if (abs(pitchFwd) > MAX_TILT * 1.2)
      newbalance = false; //way too much tilt
  }
  if (isBalancing_ != newbalance) {
    isBalancing_ = newbalance;
    D_LOG("Balancing mode %s", isBalancing_? "enabled" : "disabled");
    resetPids();
    lastBalanceChange_ = now;
    for (auto d : {right_, left_, back_}) {
      if (!d) continue;
      auto wantmode = enabled? ((isBalancing_ && d != back_)? MotorMode::Current : MotorMode::Speed) : MotorMode::Disabled;
      d->setMode(wantmode);
      D_LOG("m[%d] set mode %d", d->getName(), wantmode);
    }
  }

  //main control loop
  m.fwd  *= m.maxSpeed;
  m.side *= m.maxSpeed;
  m.yaw  *= m.maxSpeed;

  m.fwd  += state.activeCmd.d_fwd;
  m.side += state.activeCmd.d_side;
  m.yaw  += state.activeCmd.d_yaw;

  float y = -m.yaw; //convert yaw to angular rate

  float vfwd = 0, vyaw = 0;
  calcBalanceSpeeds(state, vfwd, vyaw);
  if (isBalancing_) {

    // Blend between angle-control and odometry speed control
    const uint32_t balanceChangeDuration = 2000; //ms
    float pitchGoal = balanceModeSpeed? balanceSpeedCtrl_.update(now, vfwd - m.fwd) : -m.fwd;
    pitchGoal += state.activeCmd.pitchOffsetDeg;
    if (balanceModeSpeed && (now - lastBalanceChange_) < balanceChangeDuration) {
      const float blendT = (now - lastBalanceChange_) / (float)balanceChangeDuration;
      pitchGoal = blend( -m.fwd, pitchGoal, blendT);
      if (blendT < 0.5f)
        balanceSpeedCtrl_.reset(); //prevent rampup while not in control
    }
    float torqueCmd = balanceCtrl_.update(now, pitchGoal - pitchFwd); //input is angle
    torqueCmd *= 10.0; //roughly scale to Nm
    m.fwd = torqueCmd; // Use torque output directly

    y = balanceYawCtrl_.update(now, (y + -m.side) - vyaw); //input is speed, output is torque
    m.side = 0; //disable side
    D_LOG("(fwd %06.2f vfwd%06.2f)-> [-pgoal %06.2f -p %06.2f] vyaw(%06.2f) -> %06.2f m[%d,%d,%d]",
      m.fwd, vfwd, pitchGoal, pitchFwd, vyaw, y, state.motorStates[0].mode, state.motorStates[1].mode, state.motorStates[2].mode);
  } else {
    balanceSpeedCtrl_.reset();
    balanceCtrl_.reset();
  }
  if (enabled && back_ && left_ && right_) {
    back_ ->setSetpoint(MotorMode::Speed, isBalancing_? vyaw : (y  +   0   + m.side));
    left_ ->setSetpoint(isBalancing_? MotorMode::Current : MotorMode::Speed, y  - m.fwd   - m.side * 1.33/2);
    right_->setSetpoint(isBalancing_? MotorMode::Current : MotorMode::Speed, y  + m.fwd   - m.side * 1.33/2);
  }

  status_ = isBalancing_? "woah." : enabled? "wee!" : ":|";
}

void TriOmni::updateState(uint32_t now, SharedState& state) {
  state.isBalancing = isBalancing_;
  state.dispRotate = (isBalancing_ || (abs(pitchFwd_) < MAX_TILT)) ? 0 : 2;
  state.title = status_;
}

void TriOmni::resetPids() {
  balanceCtrl_.reset();
  balanceSpeedCtrl_.reset();
  balanceYawCtrl_.reset();
}
