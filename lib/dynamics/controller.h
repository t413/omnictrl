#pragma once

#include "rfprotocol.h"
#include "displayhandler.h"
#include "behavior.h"
#include <peers.h>
#include <leds.h>
#include <WString.h>

namespace lgfx { inline namespace v1 { class LGFX_Device; } }

class OnboardingCtrl;
class DynamicsBase;
class AlfredoCRSF;
class MotorDrive;
class DriveManager;
struct SharedState;
typedef Peer<MotionControl> CPeer;

extern const uint8_t* PEER_CRSF_MAC;

class Controller {
  behavior::Manager behaviors_;
  int8_t lastBehaviorIdx_ = -1;
  PeerMgr<MotionControl> peerMgr_;
  DisplayHandler display_;
  leds::LedRig leds_;
  OnboardingCtrl* onboarding_ = nullptr;

  AlfredoCRSF* crsf_ = nullptr;
  int lastchan8_ = 0;
  bool enabled_ = false;

  static const uint8_t MAX_ADJUSTABLES = 15;
  struct Adj { float* v; String name; };
  Adj adjustables_[MAX_ADJUSTABLES] = {0};
  uint8_t adjustablesCount_ = 0;
  uint8_t selectedTune_ = MAX_ADJUSTABLES; //none selected

  SharedState* sharedState_ = nullptr;

public:
  Controller(String version);
  ~Controller();

  void addAdjustable(float* adjustable, const String& name);

  void setup(SharedState*, AlfredoCRSF* crsf);
  void setOnboarding(OnboardingCtrl* o) { onboarding_ = o; }
  void loop();
  void disable(String reason = "");

  uint8_t findPeer(const uint8_t* mac, bool allownew);
  uint8_t delegatePeer(const CPeer* old, uint32_t now); //switch to secondary controll, if possible
  CPeer* armActivate(uint8_t peeridx);
  void adjustModeBump(bool up = true, bool clear = false);

  MotionControl getCrsfCtrl(uint32_t now) const;
  void resetPids();
  bool isCrsfActive() const;
  AlfredoCRSF* getCrsf() { return crsf_; }
  bool getEnabled() const { return enabled_; }
  DisplayHandler* getDisplay() { return &display_; }
  behavior::Manager& getBehaviorMgr() { return behaviors_; }

  void send(Cmds cmd, CPeer const* peer, const uint8_t* pyld = nullptr, uint8_t len = 0);
  void broadcast(Cmds cmd, const uint8_t* pyld = nullptr, uint8_t len = 0);
  void sendInfoStr(String, CPeer const* peer = nullptr);
  void handleRxPacket(const uint8_t* mac, const uint8_t* buf, uint8_t len);

  void drawLCD(const uint32_t);
  void drawLEDs(const uint32_t now);
  const String version_;
  float lowVoltageCutoff_ = 21.0; //6S 3.5V/cell
  friend void imuControlTask(void* arg);
};
