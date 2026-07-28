#pragma once
#include <stdint.h>

class DriveManager;
class DynamicsBase;
class Controller;
class MotorDrive;

class OnboardingCtrl {
  DriveManager* mgr_ = nullptr;
  DynamicsBase* dyn_ = nullptr;
  Controller* ctrl_ = nullptr;
  MotorDrive* drive_ = nullptr;
  uint8_t selected_ = 0;
  uint32_t lastDraw_ = 0;
public:

  static OnboardingCtrl* checkCreate(DriveManager*, DynamicsBase*, Controller*);

  void iterate(uint32_t now); // Called from Controller::loop() when onboarding_ is active
  // Button inputs — Controller forwards these instead of handling them itself
  void onShortPress();
  void onPressDouble();
  void onLongPress();

  MotorDrive* selectedSlot() const;

private:
  OnboardingCtrl(DriveManager* mgr, DynamicsBase*, Controller*);
  void draw(uint32_t now);
};
