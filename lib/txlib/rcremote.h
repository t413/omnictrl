#pragma once

#include <rfprotocol.h>
#include <peers.h>
#include <displayhandler.h>
#include <WString.h>
#include <MadgwickAHRS.h>
#include <m5_unit_joystick2.hpp>
#include <esp_now.h>

namespace lgfx { inline namespace v1 { class LGFX_Device; } }

constexpr uint32_t BTN_SHORTPRESS_MAX = 500;  // milliseconds
constexpr uint32_t DISPLAY_SLEEP_MS = 5000;
constexpr uint32_t IDLE_POWEROFF_SLEEP_MS = 20000;
typedef Peer<Telem> TPeer;

class RCRemote {
  M5UnitJoystick2 joy_;
  DisplayHandler display_;
  PeerMgr<Telem> peerMgr_;
  bool armed_ = false;
  uint32_t lastWasMoved_ = 0;
  bool powerSaveMode_ = false;
  bool joystickSetup_ = false;
  bool lastJoyBtn_ = false;
  uint32_t lastJoyBtnChange_ = 0;
  uint32_t lastPoll_ = 0;
  uint32_t lastDraw_ = 0;
  MotionControl lastMotion_;
  float lastSentFailFilt_ = 1.0f;
  uint16_t sendFails_ = 0;
  uint16_t joyCenterX_ = 0, joyCenterY_ = 0;
  uint32_t lastPing_ = 0;
  float isChargingFilt_ = 0;
  bool isCharging_ = false;

public:
  RCRemote(String version);
  ~RCRemote();

  void setup();
  void setupJoystick(uint16_t maxtries = 10);
  void loop();

  void pollJoystick(uint32_t now);
  void setArmState(bool arm);
  void handleRxPacket(const uint8_t* mac, const uint8_t* buf, uint8_t len);
  void drawLCD(const uint32_t);
  bool updateIMU();
  const String version_;

  uint8_t nextPeer(uint8_t current, bool allowold);
  uint8_t validPeerCount() const;
  void send(Cmds cmd, const uint8_t* pyld = nullptr, uint8_t len = 0, bool broadcast = false);

  void setWakeupPower(bool wakeup);
};
