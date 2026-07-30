#include "onboarding.h"
#include "dynamics_base.h"
#include "controller.h"
#include <multimotor/debugprint.h>
#include <multimotor/drive_manager.h>
#include <multimotor/motordrive.h>
#include <M5Unified.h>
#include <Arduino.h>

OnboardingCtrl::OnboardingCtrl(DriveManager* mgr, DynamicsBase* dyn, Controller* ctrl) : mgr_(mgr), dyn_(dyn), ctrl_(ctrl) { }


OnboardingCtrl* OnboardingCtrl::checkCreate(DriveManager* mgr, DynamicsBase* dyn, Controller* ctrl) {
  M5.update();
  auto first = mgr->first();
  if (M5.BtnA.isPressed() && first) {
    auto ret = new OnboardingCtrl(mgr, dyn, ctrl);
    ret->drivecount_ = mgr->getCount(); //set before creating the probe drive
    DebugPrinter::setPlatformSpecific(&Serial);
    ret->drive_ = first->makeDuplicate(); //new default ID instance
    D_LOG("onboarding time! drive");
    ret->startScan(true);
    M5.Lcd.fillScreen(VIOLET); //indicate user should release
    while (M5.BtnA.isPressed()) { //wait for button release!
      M5.update();
      ret->scanIterate(); //might as well keep looking
    }
    M5.Lcd.clear();
    return ret;
  }
  return nullptr;
}

void OnboardingCtrl::bumpId() {
  auto id = drive_->getId();
  for (uint16_t i = 0; i < 253; ++i) {
    id++;
    if (id > 255) id = 0;
    if (drive_->validID(id)) {
      drive_->writeNewId(id, false);
      return;
    }
  }
}

void OnboardingCtrl::startScan(bool includeStart) {
  if (!drive_) return;
  drive_->setSetpoint(MotorMode::Speed, 0);
  drive_->setMode(MotorMode::Disabled);
  if (!includeStart) {
    bumpId();
  }
  D_LOG("start scan at 0x%x", drive_->getId());
  scanStart_ = millis(); //causes scanIterate() to keep going
}

void OnboardingCtrl::scanIterate() {
  if (!drive_ || !scanStart_) return;
  D_LOG("scan pinging drive id(%d / 0x%x)", drive_->getId(), drive_->getId());
  if (drive_->ping(50)) {
    D_LOG("scan found drive id(0x%x)", drive_->getId());
    scanStart_ = 0;
    drive_->setMode(MotorMode::Speed);
  } else {
    bumpId();
  }
}

MotorDrive* OnboardingCtrl::selectedSlot() const {
  return mgr_? mgr_->getDrives()[selected_] : nullptr;
}

constexpr auto pagebg = BLACK;

void OnboardingCtrl::clear() {
  ctrl_->getDisplay()->clearContent(pagebg, 0, true);
}

void OnboardingCtrl::draw(uint32_t now) {
  auto& disp = *(ctrl_->getDisplay());
  disp.startFrame();
  const auto pageClrA = scanStart_? DARKGREY : VIOLET;

  String title = (scanStart_? "Scan " : "Setup ") + String(drive_? drive_->getId() : -1, 16);

  disp.drawBorder(pageClrA);
  disp.setFont(&FreeSansBold12pt7b);
  disp.drawTitle(title, WHITE, pageClrA);
  disp.clearContent(pagebg, now);

  if (!scanStart_) {
    auto drvslot = selectedSlot();
    disp.setFont(&FreeSansBold9pt7b);
    disp.drawCentered("[long] set as:", pagebg);
    disp.drawCentered(drvslot? drvslot->getName() : "?", pagebg);
    String idstr = "id 0x" + String(drvslot? drvslot->getId() : -1, 16);
    disp.drawCentered(idstr.c_str(), pagebg);
  } else disp.clearContent(pagebg, now);
  disp.endFrame();
}

void OnboardingCtrl::onShortPress() {
  if (!drive_ || scanStart_) return;
  selected_ = (selected_ + 1) % drivecount_;
  auto drvslot = selectedSlot();
  D_LOG("onboarding select %d. for %d / %s", selected_, drvslot? drvslot->getId() : -1, drvslot? drvslot->getName() : "?");
  draw(millis());
}

void OnboardingCtrl::onPressDouble() { clear(); startScan(false); }

void OnboardingCtrl::onLongPress() {
  if (!drive_ || scanStart_) return;
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

uint32_t lastMoveScan = 0;

void OnboardingCtrl::iterate(uint32_t now) {
  if (now - lastDraw_ > (scanStart_? 50 : 200)) {
    lastDraw_ = now;
    draw(now);
  }
  if (drive_ && (now - lastMoveScan) > (scanStart_? 10 : 100)) {
    lastMoveScan = now;
    if (scanStart_) {
      scanIterate();
    } else {
      float pos = 4.0f * sinf((float)now / 1000.0f * 1.0f * 3.14159f);
      drive_->setSetpoint(MotorMode::Speed, pos);
    }
  }
}
