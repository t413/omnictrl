#include "onboarding.h"
#include "dynamics_base.h"
#include "controller.h"
#include <multimotor/drive_manager.h>
#include <multimotor/motordrive.h>
#include <M5Unified.h>
#include <Arduino.h>

OnboardingCtrl::OnboardingCtrl(DriveManager* mgr, DynamicsBase* dyn, Controller* ctrl) : mgr_(mgr), dyn_(dyn), ctrl_(ctrl) { }


OnboardingCtrl* OnboardingCtrl::checkCreate(DriveManager* mgr, DynamicsBase* dyn, Controller* ctrl) {
  M5.update();
  if (M5.BtnA.isPressed()) {
    auto ret = new OnboardingCtrl(mgr, dyn, ctrl);
    D_LOG("onboarding time! drive");
    while (M5.BtnA.isPressed()) { M5.update(); } //wait for release!
    return ret;
  }
  return nullptr;
}

MotorDrive* OnboardingCtrl::selectedSlot() const {
  return mgr_? mgr_->getDrives()[selected_] : nullptr;
}

constexpr auto pagebg = BLACK;
constexpr auto pageClrA = VIOLET;

void OnboardingCtrl::draw(uint32_t now) {
  auto& disp = *(ctrl_->getDisplay());
  disp.startFrame();

  String title = drive_? "Drive " + String(drive_->getId()) : "No drive!";

  disp.drawBorder(pageClrA);
  disp.setFont(&FreeSansBold12pt7b);
  disp.drawTitle(title, WHITE, pageClrA);
  disp.clearContent(pagebg, now);

  if (drive_) {
    auto drvslot = selectedSlot();
    disp.setFont(&FreeSansBold9pt7b);
    disp.drawCentered("[long] set as:", pagebg);
    disp.drawCentered(drvslot? drvslot->getName() : "?", pagebg);
    String idstr = "id 0x" + String(drvslot? drvslot->getId() : -1, 16);
    disp.drawCentered(idstr.c_str(), pagebg);
  }
  disp.endFrame();
}

void OnboardingCtrl::onShortPress() {
  if (!drive_) return;
  selected_ = (selected_ + 1) % mgr_->getCount();
  auto drvslot = selectedSlot();
  D_LOG("onboarding select %d. for %d / %s", selected_, drvslot? drvslot->getId() : -1, drvslot? drvslot->getName() : "?");
  draw(millis());
}

void OnboardingCtrl::onPressDouble() {
  //TODO iterate on the drive to change
}

void OnboardingCtrl::onLongPress() {
  if (!drive_) return;
  auto& disp = *(ctrl_->getDisplay());
  auto drvslot = selectedSlot();
  uint8_t targetId = drvslot->getId();
  D_LOG("onboarding set id %d", targetId);
  // Blocking — that's fine here
  bool ok = drive_->writeNewId(targetId);
  bool ping = ok && drive_->ping();
  if (ok && ping) {
    disp.startFrame();
    const uint32_t now = millis();
    disp.drawBorder(pagebg);
    disp.drawTitle(String("Set! ID->") + targetId, 0xFFFF, pagebg);
    disp.clearContent(pagebg, now);
    disp.drawCentered(drvslot->getName(), pagebg);
    disp.endFrame();
    delay(1500);
    ESP.restart();
  } else {
    disp.startFrame();
    const uint32_t now = millis();
    disp.drawBorder(pagebg);
    disp.drawTitle("Write failed!", 0xFFFF, pagebg);
    disp.clearContent(pagebg, now);
    disp.endFrame();
    delay(1500);
  }
}

uint32_t lastMove = 0;

void OnboardingCtrl::iterate(uint32_t now) {
  if (now - lastDraw_ > 500) {
    lastDraw_ = now;

    if (!drive_ && (drive_ = mgr_->findAtDefaultId())) {
      D_LOG("found drive! %p %d", drive_, drive_->getId());
      // drive_->setMode(MotorMode::Speed);
      drive_->setMode(MotorMode::Position);
    }

    draw(now);
  }
  if (drive_ && (now - lastMove) > 100) {
    lastMove = now;
    float pos = 6.0f * sinf((float)now / 1000.0f * 2.0f * 3.14159f);
    drive_->setSetpoint(MotorMode::Position, pos);
  }
}
