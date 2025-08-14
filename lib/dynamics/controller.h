#pragma once

#include "rfprotocol.h"
#include "displayhandler.h"
#include <WString.h>
#include <MadgwickAHRS.h>

namespace lgfx { inline namespace v1 { class LGFX_Device; } }

class DynamicsBase;
class AlfredoCRSF;
class MotorDrive;
class DriveManager;

class Controller {
  DriveManager* driveManager_ = nullptr;
  DynamicsBase* dynamics_ = nullptr;

  AlfredoCRSF* crsf_ = nullptr;
  MotionControl lastEspNowCmd_;
  MotionControl lastCrsfCmd_;
  MotionControl* activeTx_ = nullptr; //who's in control
  uint8_t remoteMac_[6] = {0}; //14:2B:2F:B0:52:A0 is tx
  bool enabled_ = false;
  Madgwick imuFilt_;
  float gyroScale_ = 1.0;

  Telem telem_;
  DisplayHandler display_;

  static const uint8_t MAX_ADJUSTABLES = 15;
  float* adjustables_[MAX_ADJUSTABLES] = {0};
  String adjNames_[MAX_ADJUSTABLES];

  uint8_t selectedTune_ = MAX_ADJUSTABLES; //none selected

public:
  Controller(String version);
  ~Controller();

  void addAdjustable(float* adjustable, const String& name);

  void setup(DynamicsBase*, DriveManager*, AlfredoCRSF* crsf);
  void loop();
  void disable();

  bool isLinkUp(uint32_t) const;
  MotionControl getCrsfCtrl(uint32_t now) const;
  uint8_t getValidDriveCount() const;
  void resetPids();
  static bool quaternionToRotationMatrix(const float q[4], float r[3][3]);
  Madgwick* getImuFilter() { return &imuFilt_; }
  MotionControl* getActiveTx() { return activeTx_; }
  bool isCrsfActive() const { return activeTx_ == &lastCrsfCmd_; }
  AlfredoCRSF* getCrsf() { return crsf_; }
  bool getEnabled() const { return enabled_; }
  Telem* getTelem() { return &telem_; }
  DisplayHandler* getDisplay() { return &display_; }
  MotorDrive* const* getDrives() const;
  uint8_t getDriveCount() const;

  void handleRxPacket(const uint8_t* mac, const uint8_t* buf, uint8_t len);
  void drawLCD(const uint32_t);
  bool updateIMU();
  const String version_;
  float lowVoltageCutoff_ = 21.0; //6S 3.5V/cell
  float gyroZ = 0.0f; //updated by IMU
};
