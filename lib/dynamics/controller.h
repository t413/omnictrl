#pragma once

#include "rfprotocol.h"
#include "displayhandler.h"
#include "behavior.h"
#include <peers.h>
#include <WString.h>
#include <MadgwickAHRS.h>

namespace lgfx { inline namespace v1 { class LGFX_Device; } }

class DynamicsBase;
class AlfredoCRSF;
class MotorDrive;
class DriveManager;

typedef Peer<MotionControl> CPeer;

extern const uint8_t* PEER_CRSF_MAC;

class Controller {
  DriveManager* driveManager_ = nullptr;
  DynamicsBase* dynamics_ = nullptr;
  behavior::Manager behaviors_;
  PeerMgr<MotionControl> peerMgr_;

  AlfredoCRSF* crsf_ = nullptr;
  bool enabled_ = false;
  Madgwick imuFilt_;
  float gyroScale_ = 1.0;
  int lastchan8_ = 0;

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

  uint8_t findPeer(const uint8_t* mac, bool allownew);
  uint8_t delegatePeer(const CPeer* old, uint32_t now); //switch to secondary controll, if possible

  MotionControl getCrsfCtrl(uint32_t now) const;
  uint8_t getValidDriveCount() const;
  void resetPids();
  static bool quaternionToRotationMatrix(const float q[4], float r[3][3]);
  Madgwick* getImuFilter() { return &imuFilt_; }
  bool isCrsfActive() const;
  AlfredoCRSF* getCrsf() { return crsf_; }
  bool getEnabled() const { return enabled_; }
  Telem* getTelem() { return &telem_; }
  DisplayHandler* getDisplay() { return &display_; }
  MotorDrive* const* getDrives() const;
  uint8_t getDriveCount() const;
  behavior::Manager& getBehaviorMgr() { return behaviors_; }
  behavior::Control getControl(uint32_t now);

  void send(Cmds cmd, CPeer const* peer, const uint8_t* pyld = nullptr, uint8_t len = 0);
  void handleRxPacket(const uint8_t* mac, const uint8_t* buf, uint8_t len);

  void drawLCD(const uint32_t);
  bool updateIMU();
  const String version_;
  float lowVoltageCutoff_ = 21.0; //6S 3.5V/cell
  float gyroZ = 0.0f; //updated by IMU
};
