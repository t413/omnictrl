#include "motiontask.h"
#include "controller.h"
#include <log.h>
#include <dynamics_base.h>
#include <multimotor/drive_manager.h>
#include <multimotor/can/cybergear.h>
#include <Arduino.h>
#include <M5Unified.h>


void imuControlTask(void* arg) {
  MotionTask* ctx = (MotionTask*)arg;
  uint32_t nextFetch = millis() + IMU_UPDATE_PERIOD;
  uint32_t nextUpdate = nextFetch + FETCH_OFFSET;
  uint32_t nextPollStats = nextFetch - IMU_UPDATE_PERIOD / 2; //get started early
  bool enabled_ = false;
  uint32_t counter = 0;

  while (1) {
    uint32_t now = millis();
    auto drives = ctx->getDrives();
    auto dcount = ctx->driveManager_->getCount();

    // run drives requestStatus() 2ms before the imu/pid/output update runs:
    if ((int32_t)(now - nextFetch) >= 0) {
      for (int i = 0; i < dcount; i++) {
        if (!drives[i]) continue;
        drives[i]->requestStatus();
        ctx->driveManager_->readOnce(now, 180); //1megabaud / 8bytes -> 80micros
      }
      nextFetch = nextUpdate + IMU_UPDATE_PERIOD - FETCH_OFFSET; //keep in sync
    }
    if ((int32_t)(now - nextUpdate) >= 0) {
      if (!M5.Imu.isEnabled()) {
        delayMicroseconds(100);
        continue;
      }
      M5.Imu.update();
      auto data = M5.Imu.getImuData();

      ctx->imuFilt_.updateIMU<0,'D'>(-data.gyro.y * ctx->gyroScale_, -data.gyro.x * ctx->gyroScale_, -data.gyro.z, data.accel.y, data.accel.x, data.accel.z); //acc x/y are swapped

      portENTER_CRITICAL(&ctx->state_.lock);
      ctx->imuFilt_.getQuaternion(ctx->state_.q);
      ctx->state_.gyroZ = -data.gyro.z;
      ctx->state_.accelX = data.accel.x;
      ctx->state_.accelY = data.accel.y;
      ctx->state_.accelZ = data.accel.z;
      ctx->state_.pitchDeg = ctx->imuFilt_.getPitchDegree();
      ctx->state_.timestamp = now;
      auto statecpy = ctx->state_; // get latest data from main thread
      portEXIT_CRITICAL(&ctx->state_.lock);

      if (ctx->dynamics_ && ctx->driveManager_) { // Update dynamics (PIDs, writes to can bus)
        ctx->driveManager_->iterate(now, 0); //handle any other incoming data
        ctx->dynamics_->iterate(now, statecpy, *ctx->driveManager_); // convert µs to ms

        bool arm = statecpy.getEnabled();
        if ((arm != enabled_)) { //state chage detect
          ctx->dynamics_->enable(arm); //changed state, notify
          enabled_ = arm;
        }
      }

      portENTER_CRITICAL(&ctx->state_.lock);
      if (ctx->dynamics_) {
        ctx->dynamics_->updateState(now, ctx->state_);
      }
      portEXIT_CRITICAL(&ctx->state_.lock);

      nextUpdate = now + IMU_UPDATE_PERIOD;
    }
    if ((int32_t)(now - nextPollStats) >= 0) {
      auto statecpy = ctx->state_.getCopy();
      uint32_t latest = 0;
      float vbus = statecpy.telem.vbus;
      for (int i = 0; i < dcount; i++) {
        if (!drives[i]) continue;
        drives[i]->fetchVBus();  // Also request VBUS parameter
        ctx->driveManager_->readOnce(now, 180);
      }

      for (int i = 0; i < dcount; i++) {
        if (!drives[i]) break;
        auto time = drives[i]->getLastStatusTime();
        latest = max(latest, time);
        auto state = drives[i]->getMotorState();
        if (!statecpy.getEnabled() && state.mode != MotorMode::Disabled)
          drives[i]->setMode(MotorMode::Disabled); //should be disabled
        if ((now - time) > 200)
          continue;
        auto v = drives[i]->getVBus();
        if (v > 0.9) { //has a value
          if (vbus < 0.1 && v > 0.1)
            vbus = v; // initialize filtered VBUS
          vbus = 0.9 * vbus + 0.1 * v; // simple low-pass filter
        }
      }
      portENTER_CRITICAL(&ctx->state_.lock);
      ctx->state_.telem.vbus = vbus;
      ctx->state_.telem.timestamp = latest;
      ctx->state_.validDriveCount = ctx->getDriveCount(true);
      for (int i = 0; i < min(dcount, MOTORS_MAX); i++)
        if (drives[i])
          ctx->state_.motorStates[i] = drives[i]->getMotorState();
      portEXIT_CRITICAL(&ctx->state_.lock);
      nextPollStats = now + POLL_STATS_UPDATE_PERIOD;
      D_LOG("fetched %d, %d, msgs[0]: t%d v%0.1f [%0.1f,%0.1f,%0.1f]", dcount, counter % 32, latest, vbus, drives[0]->getVBus(), drives[1]->getVBus(), drives[2]->getVBus());

      counter++;
    }
    delay(1);
  }
}

SharedState SharedState::getCopy() {
  portENTER_CRITICAL(&lock);
  auto ret = *this;
  portEXIT_CRITICAL(&lock);
  return ret;
}

void MotionTask::setup(DynamicsBase* dyn, DriveManager* dri, Controller* ctrl) {
  dynamics_ = dyn;
  driveManager_ = dri;
  comms_ = ctrl;
  if (M5.Imu.isEnabled()) {
    M5.Imu.loadOffsetFromNVS();
    imuFilt_.setFrequency(1000.0f / IMU_UPDATE_PERIOD); //Hz
  }
  if (dyn) dyn->init();
  D_LOG("starting motion task");
  xTaskCreatePinnedToCore(imuControlTask, "Motion", 4096, this, 3, &imuTaskHandle_, 0);
}

void MotionTask::addAdjustable(float* adjustable, const String& name) {
  comms_->addAdjustable(adjustable, name);
}

void MotionTask::resetPids() {
  if (dynamics_) {
    dynamics_->resetPids();
  }
}

MotorDrive* const* MotionTask::getDrives() const {
  return driveManager_? driveManager_->getDrives() : nullptr;
}

uint8_t MotionTask::getDriveCount(bool validonly) const {
  const uint8_t driveCount = driveManager_? driveManager_->getCount() : 0;
  if (driveCount == 0 || !driveManager_ || !validonly) return driveCount;
  uint8_t validCount = 0;
  auto drives = driveManager_->getDrives();
  for (int i = 0; i < driveCount; i++) {
    if (drives[i] == nullptr) break;
    bool hasRecent = (millis() - drives[i]->getLastStatusTime()) < 1000;
    if (hasRecent) validCount++;
  }
  return validCount;
}

void MotionTask::disable() {
  auto drives = getDrives();
  auto count = getDriveCount(false);
  for (int i = 0; i < count; i++)
    if (drives[i])
      drives[i]->setMode(MotorMode::Disabled);
  //TODO disable dynamics
}

bool MotionTask::quaternionToRotationMatrix(const float q[4], float r[3][3]) {
  // float R[3][3] = {0};
  r[0][0] = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  r[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
  r[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
  r[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
  r[1][1] = 1 - 2 * (q[1] * q[1] + q[3] * q[3]);
  r[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
  r[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
  r[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
  r[2][2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  return true;
}
