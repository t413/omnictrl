#include "uni_balancer.h"
#include <log.h>
#include <multimotor/motordrive.h>
#include <motiontask.h>
#include <utils.h>

UniBalancer::UniBalancer(MotionTask* ctrl) : DynamicsBase(ctrl) { }

void UniBalancer::init() {
  ctx_->addAdjustable(&balCtrl_.P, "b.P");
  ctx_->addAdjustable(&balCtrl_.I, "b.I");
  ctx_->addAdjustable(&balCtrl_.D, "b.D");
  ctx_->addAdjustable(&balCtrl_.rampLimit, "b.Rl");
  ctx_->addAdjustable(&speedCtrl_.P, "spd.P");
  ctx_->addAdjustable(&speedCtrl_.I, "spd.I");
  ctx_->addAdjustable(&speedCtrl_.D, "spd.D");
  ctx_->addAdjustable(&speedCtrl_.rampLimit, "spd.Rl");

  auto drives = ctx_->getDrives();
  auto dcount = ctx_->getDriveCount(false);
  for (uint8_t i = 0; i < dcount; i++) {
    if (drives[i])
      drives[i]->setMode(MotorMode::Disabled); // start with disabled mode
  }
}

MotorDrive* UniBalancer::getMotor() {
  auto drives = ctx_->getDrives();
  return drives[0];
}

void UniBalancer::enable(bool en) {
  resetPids();
  auto drives = ctx_->getDrives();
  auto dcount = ctx_->getDriveCount(false);
  for (uint8_t i = 0; i < dcount; i++)
    if (drives[i])
      drives[i]->setMode(en? MotorMode::Current : MotorMode::Disabled);
}

void UniBalancer::iterate(uint32_t now, SharedState& state, DriveManager& dm) {
  auto motor = getMotor();
  auto& motion = state.activeCmd.m;
  bool enabled = motion.state > 0;
  auto& telem = state.telem;
  const auto mstate = motor? motor->getMotorState() : MotorState();

  bool isUpOnEnd = abs(state.pitchDeg) < MAX_TILT; //more tilt allowed when balancing

  //main control loop
  if (!motor) { status_ = "no motor"; return; }

  float fwd = motion.fwd * motion.maxSpeed;

  fwdSpeed_ = mstate.velocity;
  bool balanceModeSpeed = true; //TODO make this adjustable

  if (isUpOnEnd && enabled) {
    // Blend between angle-control and odometry speed control
    const uint32_t balanceChangeDuration = 2000; //ms
    float pitchGoal = balanceModeSpeed? speedCtrl_.update(now, fwdSpeed_ - fwd) : -fwd;
    float torqueCmd = balCtrl_.update(now, pitchGoal - state.pitchDeg); //input is angle
    torqueCmd *= 10.0; //roughly scale to Nm
    D_LOG("(fwd %06.2f)-> [-pgoal %06.2f -p %06.2f] -> torque %06.2f", fwd, pitchGoal, state.pitchDeg, torqueCmd);

    motor->setSetpoint(MotorMode::Current, torqueCmd);
  } else {
    speedCtrl_.reset();
    balCtrl_.reset();
  }

  telem.pitch = state.pitchDeg; //save for next loop
}

void UniBalancer::resetPids() {
  balCtrl_.reset();
  speedCtrl_.reset();
  fwdSpeed_ = 0.0;
  status_ = "reset";
}
