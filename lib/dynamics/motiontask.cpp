#include "motiontask.h"
#include "controller.h"
#include <log.h>
#include <dynamics_base.h>
#include <multimotor/drive_manager.h>
#include <multimotor/motordrive.h>
#include <multimotor/can/cybergear.h>
#include <Arduino.h>
#include <M5Unified.h>

constexpr uint32_t IMU_UPDATE_PERIOD = 10; //ms
constexpr uint32_t FETCH_UPDATE_PERIOD = 40; //ms
constexpr uint32_t POLL_STATS_UPDATE_PERIOD = 200; //ms

void imuControlTask(void* arg) {
  MotionTask* ctx = (MotionTask*)arg;
  uint32_t lastRun = millis();
  uint32_t lastFetch = lastFetch;
  uint32_t lastPollStats = lastFetch;
  bool enabled_ = false;
  uint32_t counter = 0;

  while (1) {
    uint32_t now = millis();
    if (now - lastRun >= IMU_UPDATE_PERIOD) {
      if (!M5.Imu.isEnabled()) {
        delayMicroseconds(100);
        continue;
      }
      M5.Imu.update();
      auto data = M5.Imu.getImuData();

      ctx->imuFilt_.updateIMU<0,'D'>(-data.gyro.y * ctx->gyroScale_, -data.gyro.x * ctx->gyroScale_, -data.gyro.z, data.accel.y, data.accel.x, data.accel.z); //acc x/y are swapped

      portENTER_CRITICAL(&ctx->state_.lock);
      auto state = ctx->state_; // get latest data from main thread
      portEXIT_CRITICAL(&ctx->state_.lock);

      ctx->imuFilt_.getQuaternion(state.q);
      state.gyroZ = -data.gyro.z;
      state.accelX = data.accel.x;
      state.accelY = data.accel.y;
      state.accelZ = data.accel.z;
      state.pitchDeg = ctx->imuFilt_.getPitchDegree();
      ctx->state_.timestamp = now;

      if (ctx->dynamics_ && ctx->driveManager_) { // Update dynamics (PIDs, writes to can bus)
        ctx->driveManager_->iterate(now); //handle incoming data
        ctx->dynamics_->iterate(now, state, *ctx->driveManager_); // convert µs to ms

        bool arm = state.getEnabled();
        if ((arm != enabled_)) { //state chage detect
          ctx->dynamics_->enable(arm); //changed state, notify
          enabled_ = arm;
        }
      }

      portENTER_CRITICAL(&ctx->state_.lock);
      ctx->state_ = state; // copy back into place
      portEXIT_CRITICAL(&ctx->state_.lock);

      lastRun = now;
    }
    if ((now - lastPollStats) > POLL_STATS_UPDATE_PERIOD) {
      auto drives = ctx->getDrives();
      auto dcount = ctx->getDriveCount();
      auto statecpy = ctx->state_.getCopy();
      uint32_t latest = 0;
      float vbus = statecpy.telem.vbus;
      for (int i = 0; i < dcount; i++) {
        if (!drives[i]) break;
        auto time = drives[i]->getLastStatusTime();
        latest = max(latest, time);
        auto state = drives[i]->getMotorState();
        if (!statecpy.getEnabled() && state.mode != MotorMode::Disabled)
          drives[i]->setMode(MotorMode::Disabled); //should be disabled
        drives[i]->fetchVBus();  // Also request VBUS parameter
        if ((now - time) > 200)
          continue;
        auto v = drives[i]->getVBus();
        if (vbus < 0.1 && v > 0.1)
          vbus = v; // initialize filtered VBUS
        vbus = 0.9 * vbus + 0.1 * v; // simple low-pass filter
      }
      portENTER_CRITICAL(&ctx->state_.lock);
      ctx->state_.telem.vbus = vbus;
      ctx->state_.telem.timestamp = latest;
      ctx->state_.validDriveCount = ctx->getDriveCount(true);
      portEXIT_CRITICAL(&ctx->state_.lock);
      lastPollStats = now;

      counter++;
      if ((now - lastFetch) > (FETCH_UPDATE_PERIOD)) {
        for (int i = 0; i < dcount; i++) {
          if (drives[i]) drives[i]->requestStatus();
        }
        D_LOG("fetched %d, %d, msgs[0]: t%d v%0.1f", dcount, counter % 32, latest, vbus);
        lastFetch = now;
      }
    }
    delayMicroseconds(100);
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
