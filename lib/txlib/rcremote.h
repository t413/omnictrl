#pragma once

#include <rfprotocol.h>
#include <displayhandler.h>
#include <WString.h>
#include <MadgwickAHRS.h>
#include <m5_unit_joystick2.hpp>
#include <esp_now.h>

namespace lgfx { inline namespace v1 { class LGFX_Device; } }

class RCRemote {
  esp_now_peer_info_t peerInfo_;
  M5UnitJoystick2 joy_;
  Madgwick imuFilt_;
  DisplayHandler display_;
  bool armed_ = false;
  uint32_t lastWasMoved_ = 0;
  bool powerSaveMode_ = false;
  bool lastBtn_ = false;
  bool pitchRollOutEn_ = false;
  uint32_t lastPoll_ = 0;
  uint32_t lastDraw_ = 0;
  MotionControl lastMotion_;
  Telem lastTelemetry_;
  bool lastSentFail_ = false;

  // Rover discovery
  static const uint8_t maxRovers_ = 8;
  uint8_t discoveredRovers_[maxRovers_][6] = {0};
  uint8_t roverCount_ = 0;
  uint8_t selectedRover_ = 0;
  uint32_t lastPing_ = 0;


public:
  RCRemote(String version);
  ~RCRemote();

  void setup();
  void loop();

  void setArmState(bool arm);
  void handleRxPacket(const uint8_t* mac, const uint8_t* buf, uint8_t len);
  void drawLCD(const uint32_t);
  bool updateIMU();
  const String version_;

  int findClient(const uint8_t* mac);
  void addClient(const uint8_t* mac);
  void setTxDest(const uint8_t* mac);
  const uint8_t* getClientDest() const;
};
