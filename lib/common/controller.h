#pragma once

#include "pid.h"
#include <WString.h>
#include <AlfredoCRSF.h>
#include <MadgwickAHRS.h>
#include <motordrive.h>
#include <vector>

namespace lgfx { inline namespace v1 { class LGFX_Device; } }
constexpr uint8_t MAX_DRIVES = 8;
struct CanMessage;
class DynamicsDriver;


class Controller {
  MotorDrive* drives_[MAX_DRIVES] = {nullptr};

  AlfredoCRSF crsf_;
  int8_t lastState_ = -1;
  bool lastLinkUp_ = false;
  Madgwick imuFilt_;
  float imuRot_[3][3] = {0};
  float gyroZ = 0;
  static constexpr uint8_t DD_MAX = 3;
  std::vector<DynamicsDriver*> drivers_;
  DynamicsDriver* activeDriver_ = nullptr;
  bool redrawLCD_ = false;
  lgfx::v1::LGFX_Device* lcd_ = nullptr;

  static const uint8_t NUM_ADJUSTABLES = 4;
  float* adjustables_[NUM_ADJUSTABLES] = { &filteredFwdSpeedAlpha_, &balanceSpeedCtrl_.P, &balanceSpeedCtrl_.I, &balanceSpeedCtrl_.D};
  String adjNames_[NUM_ADJUSTABLES] = { "a", "bP", "bI", "bD" };
  uint8_t selectedTune_ = NUM_ADJUSTABLES; //none selected

public:
  Controller(String version);
  ~Controller();

  void setup();
  void loop();

  uint8_t getValidDriveCount(uint32_t validTime = 500) const;
  MotorDrive** getDrives() { return drives_; }
  MotorDrive* getDrive(uint8_t id) const;
  AlfredoCRSF& getLink() { return crsf_; }
  const float** getRot() { return imuRot_; }
  bool canPrint() const;
  MotorDrive* add(MotorDrive* drive);

  void handleCAN(const CanMessage& msg, uint32_t now);
  void drawLCD(const uint32_t);
  bool updateIMU();
  const String version_;
};

void drawCentered(const char* text, lgfx::v1::LGFX_Device*, uint16_t bg);
