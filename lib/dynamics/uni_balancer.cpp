#include "uni_balancer.h"
#include <motordrive.h>
#include <controller.h>
#include <utils.h>

UniBalancer::UniBalancer(Controller* ctrl) : DynamicsBase(ctrl) { }

void UniBalancer::init() {
  ctrl_->addAdjustable(&balCtrl_.P, "b.P");
  ctrl_->addAdjustable(&balCtrl_.I, "b.I");
  ctrl_->addAdjustable(&balCtrl_.D, "b.D");
  ctrl_->addAdjustable(&balCtrl_.rampLimit, "b.Rl");
  ctrl_->addAdjustable(&speedCtrl_.P, "spd.P");
  ctrl_->addAdjustable(&speedCtrl_.I, "spd.I");
  ctrl_->addAdjustable(&speedCtrl_.D, "spd.D");
  ctrl_->addAdjustable(&speedCtrl_.rampLimit, "spd.Rl");

  auto drives = ctrl_->getDrives();
  for (int i = 0; i < MAX_DRIVES; i++) {
    if (drives[i])
      drives[i]->setMode(MotorMode::Disabled); // start with disabled mode
  }
}

MotorDrive* UniBalancer::getMotor() {
  auto drives = ctrl_->getDrives();
  return drives[0];
}

void UniBalancer::enable(bool en) {
  resetPids();
  auto drives = ctrl_->getDrives();
  for (int i = 0; i < MAX_DRIVES; i++)
    if (drives[i])
      drives[i]->setMode(en? MotorMode::Current : MotorMode::Disabled);
}

void UniBalancer::iterate(uint32_t now) {
  auto motor = getMotor();
  auto imu = ctrl_->getImuFilter();
  auto motion = ctrl_->getActiveTx();
  auto enabled = ctrl_->getEnabled();
  auto telem = ctrl_->getTelem();

  float pitchFwd = imu->getPitchDegree();
  bool isUpOnEnd = abs(pitchFwd) < MAX_TILT; //more tilt allowed when balancing

  //main control loop
  if (!ctrl_->getDriveCount()) { status_ = "no drives"; return; }
  if (!motion) { status_ = "no motion"; return; }
  if (!motor) { status_ = "no motor"; return; }

  float fwd = motion->fwd * motion->maxSpeed;

  fwdSpeed_ = motor->getMotorState().velocity;
  bool balanceModeSpeed = true; //TODO make this adjustable

  if (isUpOnEnd && enabled) {
    // Blend between angle-control and odometry speed control
    const uint32_t balanceChangeDuration = 2000; //ms
    float pitchGoal = balanceModeSpeed? speedCtrl_.update(now, fwdSpeed_ - fwd) : -fwd;
    float torqueCmd = balCtrl_.update(now, pitchGoal - pitchFwd); //input is angle
    torqueCmd *= 10.0; //roughly scale to Nm
    Serial.printf("(fwd %06.2f)-> [-pgoal %06.2f -p %06.2f] -> torque %06.2f\n", fwd, pitchGoal, pitchFwd, torqueCmd);

    motor->setSetpoint(MotorMode::Current, -torqueCmd);
  } else {
    speedCtrl_.reset();
    balCtrl_.reset();
  }

  telem->pitch = pitchFwd; //save for next loop
}

void UniBalancer::resetPids() {
  balCtrl_.reset();
  speedCtrl_.reset();
  fwdSpeed_ = 0.0;
  status_ = "reset";
}
