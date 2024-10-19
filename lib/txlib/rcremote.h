#pragma once

#include <rfprotocol.h>
#include <WString.h>
#include <MadgwickAHRS.h>
#include <m5_unit_joystick2.hpp>
#include <esp_now.h>

namespace lgfx { inline namespace v1 { class LGFX_Device; } }

class RCRemote {
  esp_now_peer_info_t peerInfo_;
  M5UnitJoystick2 joy_;
  Madgwick imuFilt_;
  lgfx::v1::LGFX_Device* lcd_ = nullptr;
  bool armed_ = false;
  bool lastBtn_ = false;
  bool pitchRollOutEn_ = false;
  bool redrawLCD_ = false;
  uint32_t lastPoll_ = 0;
  uint32_t lastDraw_ = 0;
  uint32_t lastClear_ = 0;
  MotionControl lastMotion_;

public:
  RCRemote(String version);
  ~RCRemote();

  void setup();
  void loop();

  void handleRxPacket(const uint8_t* buf, uint8_t len);
  void drawLCD(const uint32_t);
  bool updateIMU();
  const String version_;
};

void drawCentered(const char* text, lgfx::v1::LGFX_Device*, uint16_t bg);
