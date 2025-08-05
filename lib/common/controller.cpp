#include "controller.h"
#include <dynamics_base.h>
#include <AlfredoCRSF.h>
#include <can/can_esp32_twai.h>
#include <motordrive.h>
#include "utils.h"
#include <Arduino.h>
#ifdef IS_M5
#include <M5Unified.h>
#endif
#include <esp_now.h>
#include <WiFi.h>

constexpr uint32_t IMU_UPDATE_PERIOD = 20; //ms

bool canPrint() {
  #if ARDUINO_USB_CDC_ON_BOOT
  return Serial && Serial.availableForWrite();
  #else
  return true;
  #endif
}

bool validMac(const uint8_t* mac) {
  for (int i = 0; i < 6; i++)
    if (mac[i] != 0) return true;
  return false;
}

// -------------------- //
// ---- Controller ---- //
// -------------------- //

Controller::~Controller() { }

Controller::Controller(String version) :
        version_(version) {
  imuFilt_.setFrequency(1000 / IMU_UPDATE_PERIOD); //50Hz, 20ms
}

static Controller* controller_ = nullptr;

void Controller::addDrive(MotorDrive* drive) {
  for (int i = 0; i < MAX_DRIVES; i++) {
    if (drives_[i] == nullptr) {
      drives_[i] = drive;
      return;
    }
  }
  Serial.println("No space for new drive");
}

void Controller::addAdjustable(float* adjustable, const String& name) {
  for (int i = 0; i < MAX_ADJUSTABLES; i++) {
    if (adjustables_[i] == nullptr) {
      adjustables_[i] = adjustable;
      adjNames_[i] = name;
      return;
    }
  }
  Serial.println("No space for new adjustable");
}

void Controller::setup(DynamicsBase* dynamics) {
  controller_ = this;
  dynamics_ = dynamics;
#ifdef CONFIG_IDF_TARGET_ESP32
  Serial.begin(115200);
#endif

    Serial.begin(115200);
    Serial.setTimeout(10); //very fast, need to keep the ctrl loop running
  if (canPrint()) { //only use the virtual serial port if it's available
    Serial.println("Controller setup");
    Serial.flush();
  }
  delay(100);

  if (canPrint())
    Serial.printf("finished CAN setup\n");

  WiFi.mode(WIFI_STA);
  auto res = esp_now_init();
  if (res != ESP_OK && canPrint())
    Serial.printf("ESP-NOW init failed: %d\n", res);
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    if (controller_)
      controller_->handleRxPacket(mac, data, len);
  });

#ifdef IS_M5
  M5.begin();
  //check if has lcd
  if (M5.Lcd.width() > 0) {
    display_.setLCD(&M5.Lcd);
    M5.Lcd.setRotation(2); //flip
  }
  M5.Power.begin();
  if (M5.Imu.isEnabled()) {
    M5.Imu.loadOffsetFromNVS();
    // M5.Imu.setAxisOrderRightHanded( ??
    imuFilt_.setFrequency(50); //50Hz, 20ms
  }
#endif

  if (canPrint())
    Serial.println("finished setup");

  delay(100);
}

bool Controller::isLinkUp(uint32_t now) const {
  return (crsf_ && crsf_->isLinkUp()) || ((now - lastEspNowCmd_.timestamp) < 500);
}

MotionControl Controller::getCrsfCtrl(uint32_t now) const {
  MotionControl ret = {};
  if (crsf_ && crsf_->isLinkUp()) { //stil use crsf for speed control
    ret.state = crsf_->getChannel(5) > 1500? 1 : 0;
    ret.fwd = deadband(mapfloat(crsf_->getChannel(2), 1000, 2000, -1, 1));
    ret.side = deadband(mapfloat(crsf_->getChannel(1), 1000, 2000, -1, 1));
    ret.yaw = deadband(mapfloat(crsf_->getChannel(4), 1000, 2000, -1, 1));
    ret.adjust = mapfloat(crsf_->getChannel(3), 1000, 2000, 0.0, 1.0);
    ret.maxSpeed = mapfloat(crsf_->getChannel(7), 1000, 2000, 6, 30); //aux 2: speed selection
    ret.timestamp = now;
  }
  return ret;
}

uint8_t Controller::getValidDriveCount() const {
  uint8_t validCount = 0;
  for (int i = 0; i < MAX_DRIVES; i++) {
    if (drives_[i] == nullptr) break;
    bool hasRecent = (millis() - drives_[i]->getLastStatusTime()) < 1000;
    if (hasRecent) validCount++;
  }
  return validCount;
}

void Controller::resetPids() {
  if (dynamics_) {
    dynamics_->resetPids();
  }
}

uint8_t Controller::getDriveCount() const {
  for (int i = 0; i < MAX_DRIVES; i++)
    if (!drives_[i])
      return i;
  return MAX_DRIVES;
}

void Controller::handleRxPacket(const uint8_t* mac, const uint8_t* buf, uint8_t len) {
  auto cmd = len >= 1? (Cmds) buf[0] : Cmds::None;
  if (cmd == Cmds::MotionControl && len == (1 + sizeof(MotionControl))) {
    MotionControl rxmc = *((const MotionControl*) (buf + 1));
    rxmc.timestamp = millis();
    // Serial.printf("MC: a%d fwd %06.2f yaw %06.2f side %06.2f\n", rxmc.state, rxmc.fwd, rxmc.yaw, rxmc.side);
    if (rxmc.state > 0 && lastEspNowCmd_.state == 0) {
      Serial.println("ESPNow arm");
      activeTx_ = &lastEspNowCmd_; //take control
    }
    lastEspNowCmd_ = rxmc;

    if (memcmp(mac, remoteMac_, 6) != 0) {
      if (canPrint())
        Serial.printf("New telemetry target: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      memcpy(remoteMac_, mac, 6);
      // Register peer
      esp_now_peer_info_t newpeer = {0};
      memcpy(newpeer.peer_addr, remoteMac_, 6);
      newpeer.channel = 0;
      newpeer.encrypt = false;
      auto res = esp_now_add_peer(&newpeer);
      if (res != ESP_OK && canPrint())
        Serial.printf("ESP-NOW add peer failed: %d\n", res);
    }
  }
}

void Controller::disable() {
  for (int i = 0; i < MAX_DRIVES; i++)
    if (drives_[i])
      drives_[i]->setMode(MotorMode::Disabled);
  //TODO disable dynamics
}

bool Controller::quaternionToRotationMatrix(const float q[4], float r[3][3]) {
  // float R[3][3] = {0};
  r[0][0] = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  r[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
  r[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
  r[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
  r[1][1] = 1 - 2 * (q[1] * q[1] + q[3] * q[3]);
  r[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
  r[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
  r[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
  r[2][2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  return true;
}

uint32_t lastDraw = 0;
uint32_t lastDrive = 1000;
uint32_t lastPollStats = 0;
uint32_t lastRefreshDrives = 0;
uint32_t lastTxStats = 0;
uint32_t lastClear = 0;

void Controller::loop() {
  uint32_t now = millis();

  if ((now - lastPollStats) > 200) {
    uint32_t latest = 0;
    for (int i = 0; i < MAX_DRIVES; i++) {
      if (!drives_[i]) break;
      auto time = drives_[i]->getLastStatusTime();
      latest = max(latest, time);
      auto state = drives_[i]->getMotorState();
      if (!enabled_ && state.mode != MotorMode::Disabled)
        drives_[i]->setMode(MotorMode::Disabled); //should be disabled
      drives_[i]->fetchVBus();  // Also request VBUS parameter
      if ((now - time) > 200)
        continue;
      auto v = drives_[i]->getVBus();
      if (telem_.vbus < 0.1 && v > 0.1)
        telem_.vbus = v; // initialize filtered VBUS
      telem_.vbus = 0.9 * telem_.vbus + 0.1 * v; // simple low-pass filter
    }
    if (telem_.vbus < (lowVoltageCutoff_ - 0.2) && enabled_) {
      disable(); //disable if voltage is too low
    }
    telem_.timestamp = latest;

    // Send telemetry to remote
    if (validMac(remoteMac_)) {
      uint8_t data[1 + sizeof(Telem)] = {0};
      data[0] = (uint8_t) Cmds::Telemetry;
      memcpy(data + 1, &telem_, sizeof(Telem));
      esp_now_send(remoteMac_, data, sizeof(data));
    }

    lastPollStats = now;
    // Serial.println("MAC: " + WiFi.macAddress());
  }
  if (((now - lastTxStats) > 100) && crsf_ && crsf_->isLinkUp()) {
    crsf_sensor_battery_t crsfBatt = {
      .voltage = htobe16((uint16_t)(telem_.vbus * 10.0)),
      .current = htobe16((uint16_t)(0 * 10.0)),
      .capacity = htobe16((uint16_t)(0)) << 8,
      .remaining = (uint8_t)(0),
    };
    crsf_->queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
    lastTxStats = now;
  }

  #ifdef IS_M5
  M5.update(); //updates buttons, etc
  if (M5.BtnA.wasPressed()) {
    selectedTune_ = (selectedTune_ + 1) % (MAX_ADJUSTABLES + 1); //+1 for disabled
    if (selectedTune_ < MAX_ADJUSTABLES) {
      strncpy(telem_.adjustSrc, adjNames_[selectedTune_].c_str(), sizeof(telem_.adjustSrc) - 1);
    } else {
      telem_.adjustSrc[0] = '\0';
    }
    display_.requestRedraw();
  }
  #endif
  float* tunable = selectedTune_ < MAX_ADJUSTABLES? adjustables_[selectedTune_] : NULL;

  if ((now - lastRefreshDrives) > (IMU_UPDATE_PERIOD - 2)) {
    for (int i = 0; i < MAX_DRIVES; i++) {
      if (drives_[i]) drives_[i]->requestStatus();
    }
    lastRefreshDrives = now; //also updated below, to keep in sync with IMU updates
  }

  if ((now - lastDrive) > IMU_UPDATE_PERIOD) {
    updateIMU();

    //control inputs
    bool arm = enabled_;
    // --- CRSF Rx --- //

    auto newctrl = getCrsfCtrl(now);
    if (newctrl.state && lastCrsfCmd_.state == 0) {
      activeTx_ = &lastCrsfCmd_; //take control
      Serial.printf("CRSF arm\n");
    }
    lastCrsfCmd_ = newctrl;
    bool stale = activeTx_ && ((int32_t)(now - activeTx_->timestamp) > 1000);

    if (activeTx_ && (!activeTx_->state || stale)) { //disarmed active ctrl or disconnected
      MotionControl* otherCtrl = (activeTx_ == &lastCrsfCmd_) ? &lastEspNowCmd_ : &lastCrsfCmd_;
      if (otherCtrl->state > 0) { //delegate
        activeTx_ = otherCtrl;
        Serial.printf("Delegating to %s, stale %d\n", (activeTx_ == &lastCrsfCmd_)? "CRSF" : "ESP-NOW", stale);
      } else {
        Serial.printf("Disarming, no active control, stale %d dt %ld\n", stale, now - activeTx_->timestamp);
        activeTx_ = nullptr; //disarm
      }
    }

    if (isCrsfActive()) { //crsf control has extra features
      bool enableAdjustment = arm && crsf_ && (crsf_->getChannel(8) > 1500);
      if (enableAdjustment && tunable) {
        *tunable = pow(10, mapfloat(activeTx_->adjust, 0, 1, -2, 2));
        if (activeTx_->adjust < 0.01) *tunable = 0; //allow 0 values
      }
    } else if (activeTx_ == &lastEspNowCmd_) {
      lastEspNowCmd_.maxSpeed = 18;
    }
    arm = activeTx_ && (activeTx_->state > 0);
    if ((arm != enabled_)) {
      display_.requestRedraw();
      if (dynamics_)
        dynamics_->enable(arm); //changed state, notify
    }
    enabled_ = arm;
    if (dynamics_) {
      dynamics_->iterate(now);
    }

    lastDrive = now;
    lastRefreshDrives = now; //keep drive refresh in sync with this task
  } //IMU timing

  if ((now - lastDraw) > 60 || display_.isRedrawRequired()) {
    telem_.adjusting = tunable? *tunable : 0.0;
    drawLCD(now);
    lastDraw = now;
  }

  if (crsf_)
    crsf_->update();

  while (canInterface_ && canInterface_->available()) {
    CanMessage message = canInterface_->readOne();
    bool handled = false;
    for (int i = 0; i < MAX_DRIVES; i++) {
      if (drives_[i] && drives_[i]->handleIncoming(message.id, message.data, message.len, now))
        handled = true;
    }
    if (!handled && Serial) {
      Serial.printf("unhandled message id=%x len=%d: {", message.id, message.len);
      for (int i = 0; i < message.len; i++)
        Serial.printf("0x%02x, ", message.data[i]);
      Serial.println("}");
    }
  }
}

bool Controller::updateIMU() {
#ifdef IS_M5
  auto res = M5.Imu.isEnabled()? M5.Imu.update() : 0;
  if (!res) return false;
  auto data = M5.Imu.getImuData(); //no mag data it seems, sadly
  gyroZ = -data.gyro.z;
  imuFilt_.updateIMU<0,'D'>(-data.gyro.y * gyroScale_, -data.gyro.x * gyroScale_, -data.gyro.z, data.accel.y, data.accel.x, data.accel.z); //acc x/y are swapped
#endif
  return true;
}

void Controller::drawLCD(const uint32_t now) {
  display_.startFrame();
  auto bgRainbow = display_.timeRainbow(now);
  auto fg = BLACK;
  auto pageBG = BLACK;
  auto validCount = getValidDriveCount();
  String title = dynamics_? dynamics_->getStatus() : "?";
  String err;
  if (telem_.vbus > 0.1 && telem_.vbus < lowVoltageCutoff_) err = "low batt!";
  else if (!validCount) err = "no drives!";
  else if (!isLinkUp(now)) err = "NO LINK";
  if (!err.isEmpty()) {
    title = err;
    fg = WHITE;
    pageBG = SUPERDARKRED;
    bgRainbow = RED;
  }

  display_.setFont(&FreeSansBold12pt7b);
  display_.drawBorder(bgRainbow);
  display_.drawTitle(title.c_str(), fg, bgRainbow);
  display_.clearContent(pageBG, now);
  display_.drawTelem(telem_, now, pageBG);
  display_.drawVersion(version_, pageBG);
  display_.endFrame();
}
