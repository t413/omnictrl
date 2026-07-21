#pragma once

#include <rfprotocol.h>
#include <behavior.h>
#include <FreeRTOS.h>
#include <multimotor/motordrive.h>
#include <MadgwickAHRS.h>

class DriveManager;
class DynamicsBase;
class MotorDrive;

constexpr uint32_t IMU_UPDATE_PERIOD = 20; //ms
constexpr uint32_t FETCH_OFFSET = 3; //ms before IMU_UPDATE
constexpr uint32_t POLL_STATS_UPDATE_PERIOD = 200; //ms

constexpr uint8_t CRSF_CHANS = 10;
constexpr uint8_t MOTORS_MAX = 4;

struct SharedState {
  float gyroZ = 0.0f, accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f, pitchDeg = 0.0f;;
  uint32_t timestamp = 0;
  float q[4] = {1, 0, 0, 0};
  behavior::Control activeCmd;
  bool crsfActive = false;
  int crsfChans[CRSF_CHANS] = {0};
  uint8_t dispRotate = 0;
  uint8_t validDriveCount = 0;
  bool isBalancing = false;
  portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
  Telem telem;
  String title;
  MotorState motorStates[MOTORS_MAX]; //also vbus is in telem.vbus

  SharedState getCopy();
  bool getEnabled() const { return activeCmd.m.state > 0; }
};

void imuControlTask(void* arg);

class MotionTask {
  SharedState state_;
  Madgwick imuFilt_;

  DriveManager* driveManager_ = nullptr;
  DynamicsBase* dynamics_ = nullptr;
  Controller* comms_ = nullptr;
  TaskHandle_t imuTaskHandle_ = nullptr;
  float gyroScale_ = 1.0;

public:
  MotionTask() { }
  ~MotionTask() = default;

  void setup(DynamicsBase*, DriveManager*, Controller*);

  Madgwick* getImuFilter() { return &imuFilt_; }
  MotorDrive* const* getDrives() const;
  uint8_t getDriveCount(bool validonly = false) const;
  const SharedState& getState() const { return state_; }
  SharedState& getState() { return state_; }
  static bool quaternionToRotationMatrix(const float q[4], float r[3][3]);

  void addAdjustable(float* adjustable, const String& name);

  void resetPids();
  void disable();


  friend void imuControlTask(void* arg);
};
