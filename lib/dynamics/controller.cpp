#include "controller.h"
#include <log.h>
#include <NowPacket.h>
#include <dynamics_base.h>
#include <AlfredoCRSF.h>
#include <multimotor/drive_manager.h>
#include <multimotor/can/can_esp32_twai.h>
#include <multimotor/motordrive.h>
#include "utils.h"
#include <Arduino.h>
#ifdef IS_M5
#include <M5Unified.h>
#endif
#include <esp_now.h>
#include <WiFi.h>

constexpr uint32_t IMU_UPDATE_PERIOD = 10; //ms
const uint8_t* PEER_CRSF_MAC = BROADCAST_ADDRESS;

constexpr uint8_t PKT_SUBT = (uint8_t) Subt::Robot;

bool canPrint() {
  #if ARDUINO_USB_CDC_ON_BOOT
  return Serial && Serial.availableForWrite();
  #else
  return true;
  #endif
}

// -------------------- //
// ---- Controller ---- //
// -------------------- //

Controller::~Controller() { }

Controller::Controller(String version) :
        version_(version) { }

static Controller* controller_ = nullptr;

void Controller::addAdjustable(float* adjustable, const String& name) {
  for (int i = 0; i < MAX_ADJUSTABLES; i++) {
    if (adjustables_[i] == nullptr) {
      adjustables_[i] = adjustable;
      adjNames_[i] = name;
      return;
    }
  }
  D_LOG("No space for new adjustable");
}

void imuControlTask(void* arg) {
  Controller* ctrl = (Controller*)arg;
  uint32_t lastRun = micros();
  constexpr uint32_t PERIOD_US = IMU_UPDATE_PERIOD * 1000;

  while (1) {
    uint32_t now = micros();
    if (now - lastRun >= PERIOD_US) {
      if (!M5.Imu.isEnabled()) {
        delayMicroseconds(100);
        continue;
      }
      M5.Imu.update();
      auto data = M5.Imu.getImuData();

      ctrl->imuFilt_.updateIMU<0,'D'>(-data.gyro.y * ctrl->gyroScale_, -data.gyro.x * ctrl->gyroScale_, -data.gyro.z, data.accel.y, data.accel.x, data.accel.z); //acc x/y are swapped

      portENTER_CRITICAL(&ctrl->ctrlState_.lock);
      auto cmd = ctrl->ctrlState_.activeCmd; // get latest setpoint from main thread
      portEXIT_CRITICAL(&ctrl->ctrlState_.lock);

      if (ctrl->dynamics_ && ctrl->driveManager_) { // Update dynamics (PIDs, writes to can bus)
        ctrl->dynamics_->iterate(now / 1000, cmd, ctrl->imuFilt_, *ctrl->driveManager_); // convert µs to ms
        ctrl->driveManager_->iterate(now / 1000);
      }

      // 6. Write IMU state back for main thread
      portENTER_CRITICAL(&ctrl->ctrlState_.lock);
      ctrl->ctrlState_.gyroZ = -data.gyro.z;
      ctrl->ctrlState_.accelX = data.accel.x;
      ctrl->ctrlState_.accelY = data.accel.y;
      ctrl->ctrlState_.accelZ = data.accel.z;
      ctrl->ctrlState_.timestamp = now / 1000;
      portEXIT_CRITICAL(&ctrl->ctrlState_.lock);

      lastRun = now;
    }
    delayMicroseconds(100);
  }
}

void Controller::setup(DynamicsBase* dynamics, DriveManager* mgr, AlfredoCRSF* crsf) {
  controller_ = this;
  dynamics_ = dynamics;
  driveManager_ = mgr;
  crsf_ = crsf;
#ifdef CONFIG_IDF_TARGET_ESP32
  Serial.begin(115200);
#endif

    Serial.begin(115200);
    Serial.setTimeout(10); //very fast, need to keep the ctrl loop running
  if (canPrint()) { //only use the virtual serial port if it's available
    D_LOG("Controller setup");
    Serial.flush();
  }
  delay(100);

  if (canPrint())
    D_LOG("finished CAN setup");

  WiFi.mode(WIFI_STA);
  auto res = esp_now_init();
  if (res != ESP_OK && canPrint())
    D_LOG("ESP-NOW init failed: %d", res);
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    if (controller_)
      controller_->handleRxPacket(mac, data, len);
  });
  espnowRegisterMac(BROADCAST_ADDRESS);

  M5.begin();
  //check if has lcd
  if (M5.Lcd.width() > 0) {
    display_.setLCD(&M5.Lcd);
    M5.Lcd.setRotation(2); //flip
  }
  M5.Power.begin();
  if (M5.Imu.isEnabled()) {
    M5.Imu.loadOffsetFromNVS();
    imuFilt_.setFrequency(1000.0f / IMU_UPDATE_PERIOD); //Hz
  }

  if (canPrint())
    D_LOG("finished setup");

  delay(100);
  D_LOG("starting imu task");
  xTaskCreatePinnedToCore(imuControlTask, "IMUCtrl", 4096, this, 3, &imuTaskHandle_, 0);
  leds_.setup();
}

bool Controller::isCrsfActive() const {
  return crsf_ && crsf_->isLinkUp() && peerMgr_.activePeer_ < PEERS_MAX && peerMgr_.isActive(peerMgr_.findPeerIdx(PEER_CRSF_MAC));
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
  const uint8_t driveCount = getDriveCount();
  if (driveCount == 0) return 0;
  auto drives = driveManager_->getDrives();

  for (int i = 0; i < driveCount; i++) {
    if (drives[i] == nullptr) break;
    bool hasRecent = (millis() - drives[i]->getLastStatusTime()) < 1000;
    if (hasRecent) validCount++;
  }
  return validCount;
}

void Controller::resetPids() {
  if (dynamics_) {
    dynamics_->resetPids();
  }
}

MotorDrive* const* Controller::getDrives() const {
  return driveManager_? driveManager_->getDrives() : nullptr;
}

uint8_t Controller::getDriveCount() const {
  return driveManager_? driveManager_->getCount() : 0;
}

behavior::Control Controller::getControl(uint32_t now) {
  auto pidx = peerMgr_.getActiveIdx();
  auto p = peerMgr_.findPeer(pidx);
  return behaviors_.iterate(now, (p? p->lastCmd_ : MotionControl()), dynamics_->isBalancing(), behaviors_);
}

uint8_t Controller::delegatePeer(const CPeer* old, uint32_t now) {
  for (uint8_t i = 0; i < PEERS_MAX; i++) {
    auto& peer = peerMgr_.peers_[i];
    if (old && &peer == old) continue;
    if (peer.isValid() && peer.isRecent(now) && peer.lastCmd_.state > 0)
      return i;
  }
  return PEERS_MAX;
}

CPeer* Controller::armActivate(uint8_t peeridx) {
  auto prev = peerMgr_.findPeer(peerMgr_.activePeer_);
  auto ret = peerMgr_.findPeer(peeridx);
  //TODO safety checks. sticks centered?
  if (ret) { //arming
    const auto& m = ret->lastCmd_;
    for (float f : {m.fwd, m.side, m.yaw}) {
      if (abs(f) > 0.4) {
        D_LOG("arm-activate SAFETY CHECK FAIL (peer[%d])", peeridx);
        sendInfoStr("center ctrls", ret);
        return nullptr;
      }
    }
    D_LOG("arm-activate peer[%d]", peeridx);
    peerMgr_.activate(peeridx);
    if (prev) sendInfoStr("dropped"); //broadcast first
    sendInfoStr("ACTIVE", ret);
  } else {
    D_LOG("disarm (from[%d] to %d)", peerMgr_.activePeer_, peeridx);
    disable("disarm");
  }
  return ret;
}

void Controller::handleRxPacket(const uint8_t* mac, const uint8_t* inbuf, uint8_t inlen) {
  comms::NowPacket pkt;
  if (!pkt.parse(inbuf, inlen)) {
    D_LOG("unknown pkt data len %d:", inlen);
    return printBuf(inbuf, inlen);
  }
  auto cmd = (Cmds)pkt.type;
  auto peeridx = peerMgr_.findOrMakePeer(mac, true, true, false);
  auto peer = peerMgr_.findPeer(peeridx);
  if (!peer) {
    D_LOG("handleRx no peer");
    return;
  }

  if (cmd == Cmds::MotionControl && pkt.payloadLen == MOTION_CONTROL_SIZE) {
    MotionControl rxmc = *((const MotionControl*) pkt.payload);
    rxmc.timestamp = millis();

    if (rxmc.state > 0 && peer->lastCmd_.state == 0) {
      armActivate(peeridx);
    }
    peer->lastCmd_ = rxmc;

  } else if (cmd == Cmds::Ping) {
    peer->lastPing_ = millis();
    // Reply to ping with our MAC address
    send(Cmds::PingReply, peer);
    D_LOG("Ping received from peer[%d], replied", peeridx);
  } else if (cmd == Cmds::ModeChange) {
    behaviors_.increment();
  }
}

void Controller::send(Cmds cmd, CPeer const* peer, const uint8_t* pyld, uint8_t len) {
  if (peer && peer->isMac(PEER_CRSF_MAC)) return; //skip sending, not a esp-now target
  uint8_t txBuf[comms::HEADER_OVERHEAD + len] = {0};
  uint8_t txLen = comms::NowPacket::serialise((uint8_t)cmd, PKT_SUBT, pyld, len, txBuf, sizeof(txBuf));
  if (txLen > 0 && peer) {
    auto result = esp_now_send(peer->mac, txBuf, txLen);
  } else D_LOG("tx can't send %p %d", peer, txLen);
}

void Controller::broadcast(Cmds cmd, const uint8_t* pyld, uint8_t len) {
  uint8_t txBuf[comms::HEADER_OVERHEAD + len] = {0};
  uint8_t txLen = comms::NowPacket::serialise((uint8_t)cmd, PKT_SUBT, pyld, len, txBuf, sizeof(txBuf));
  if (txLen == 0) { D_LOG("txlen 0"); return; }
  auto result = esp_now_send(BROADCAST_ADDRESS, txBuf, txLen);
}

void Controller::sendInfoStr(String s, CPeer const* peer) {
  if (peer) send(Cmds::InfoStr, peer, (const uint8_t*) s.c_str(), s.length());
  else broadcast(Cmds::InfoStr, (const uint8_t*) s.c_str(), s.length());
}

void Controller::disable(String reason) {
  auto drives = getDrives();
  auto count = getDriveCount();
  for (int i = 0; i < count; i++)
    if (drives[i])
      drives[i]->setMode(MotorMode::Disabled);
  //TODO disable dynamics
  peerMgr_.activate(PEERS_MAX);
  sendInfoStr(reason.isEmpty()? "disable" : reason);
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

void Controller::loop() {
  uint32_t now = millis();
  auto drives = getDrives();
  auto dcount = getDriveCount();

  if ((now - lastPollStats) > 200) {
    uint32_t latest = 0;
    for (int i = 0; i < dcount; i++) {
      if (!drives[i]) break;
      auto time = drives[i]->getLastStatusTime();
      latest = max(latest, time);
      auto state = drives[i]->getMotorState();
      if (!enabled_ && state.mode != MotorMode::Disabled)
        drives[i]->setMode(MotorMode::Disabled); //should be disabled
      drives[i]->fetchVBus();  // Also request VBUS parameter
      if ((now - time) > 200)
        continue;
      auto v = drives[i]->getVBus();
      if (telem_.vbus < 0.1 && v > 0.1)
        telem_.vbus = v; // initialize filtered VBUS
      telem_.vbus = 0.9 * telem_.vbus + 0.1 * v; // simple low-pass filter
    }
    if (telem_.vbus < (lowVoltageCutoff_ - 0.2) && enabled_) {
      disable("low v"); //disable if voltage is too low
    }
    telem_.timestamp = latest;

    // broadcast telemetry
    broadcast(Cmds::Telemetry, (uint8_t*)&telem_, sizeof(Telem));

    lastPollStats = now;
    // Serial.println("MAC: " + WiFi.macAddress());
  }
  if (((now - lastTxStats) > 100)) {
    if (crsf_ && crsf_->isLinkUp()) {
      crsf_sensor_battery_t crsfBatt = {
        .voltage = htobe16((uint16_t)(telem_.vbus * 10.0)),
        .current = htobe16((uint16_t)(0 * 10.0)),
        .capacity = (uint16_t)(htobe16((uint16_t)(0)) << 8),
        .remaining = (uint8_t)(0),
      };
      crsf_->queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
      auto name = behaviors_.getName();
      crsf_->queuePacket(CRSF_SYNC_BYTE, 0x21, name, strlen(name) + 1); //CRSF_FRAMETYPE_FLIGHT_MODE
    }

    if (lastBehaviorIdx_ != behaviors_.activeIdx_) {
      sendInfoStr(String("m: ") + behaviors_.getName());
      D_LOG("mode change %s", behaviors_.getName());
      lastBehaviorIdx_ = behaviors_.activeIdx_;
    }

    lastTxStats = now;
  }

  #ifdef IS_M5
  M5.update(); //updates buttons, etc
  if (M5.BtnA.wasSingleClicked()) {
    if (enabled_) behaviors_.increment();
    else if (selectedTune_ < MAX_ADJUSTABLES) selectedTune_ = MAX_ADJUSTABLES; //clear
    else sendInfoStr("poke");
  } else if (M5.BtnA.wasDoubleClicked()) {
    selectedTune_ = (selectedTune_ + 1) % (MAX_ADJUSTABLES + 1); //+1 for disabled
    if (selectedTune_ < MAX_ADJUSTABLES) {
      sendInfoStr("adj:" + adjNames_[selectedTune_]);
    } else {
      sendInfoStr("adj-none");
    }
    display_.requestRedraw();
  }
  #endif
  float* tunable = selectedTune_ < MAX_ADJUSTABLES? adjustables_[selectedTune_] : NULL;

  if ((now - lastRefreshDrives) > (IMU_UPDATE_PERIOD - 2)) {
    for (int i = 0; i < dcount; i++) {
      if (drives[i]) drives[i]->requestStatus();
    }
    lastRefreshDrives = now; //also updated below, to keep in sync with IMU updates
  }

  if ((now - lastDrive) > IMU_UPDATE_PERIOD) {
    // IMU reading, dynamics iterate, drive iterate now done in separate thread in imuControlTask
    //control inputs
    bool arm = enabled_;

    // --- CRSF Rx --- //
    auto crsfCtrl = getCrsfCtrl(now);
    auto crsfIdx = PEERS_MAX;
    if (crsfCtrl.timestamp) { //valid!
      crsfIdx = peerMgr_.findOrMakePeer(PEER_CRSF_MAC, true, false, false);
      auto cpeer = peerMgr_.findPeer(crsfIdx);
      if (crsfCtrl.state && cpeer->lastCmd_.state == 0) {
        armActivate(crsfIdx); //take control
      }
      cpeer->lastCmd_ = crsfCtrl;
    }

    auto aidx = peerMgr_.getActiveIdx();
    auto active = peerMgr_.findPeer(aidx);
    auto stale = active? active->getStale(now) : 1337;

    if (active && (!active->lastCmd_.state || stale > 0)) { //disarmed active ctrl or disconnected
      D_LOG("disarm/stale peer[%d] state %d, stale %d", aidx, active->lastCmd_.state, stale);
      aidx = delegatePeer(active, now);
      active = armActivate(aidx);
      if (active) {
        D_LOG("Delegated to [%d], (stale %d)", aidx, stale);
      }
    }

    if (peerMgr_.isActive(crsfIdx)) { //crsf control has extra features
      const auto chan8 = crsf_? crsf_->getChannel(8) : 0;
      bool enableAdjustment = arm && crsf_ && (chan8 > 1900);
      if (enableAdjustment && tunable) {
        *tunable = pow(10, mapfloat(active->lastCmd_.adjust, 0, 1, -2, 2));
        if (active->lastCmd_.adjust < 0.01) *tunable = 0; //allow 0 values
      } else if (chan8 > 1500 && chan8 < 1900 && lastchan8_ < 1500) { //behavior bump
        behaviors_.increment();
      } else if (chan8 < 1100) {
        behaviors_.clear();
      }
      lastchan8_ = chan8;
    } else if (active) {
      active->lastCmd_.maxSpeed = 18;
    }
    arm = active && (active->lastCmd_.state > 0);
    if ((arm != enabled_)) {
      // -- ENABLING, arming, whatever -- //
      display_.requestRedraw();
      behaviors_.clear();
      if (dynamics_)
        dynamics_->enable(arm); //changed state, notify
    }
    enabled_ = arm;

    auto control = getControl(now); //calculates behavior motion

    portENTER_CRITICAL(&ctrlState_.lock);
    ctrlState_.activeCmd = control; //write for the controll thread to read
    portEXIT_CRITICAL(&ctrlState_.lock);

    lastDrive = now;
    lastRefreshDrives = now; //keep drive refresh in sync with this task
  } //IMU timing

  if ((now - lastDraw) > 60 || display_.isRedrawRequired()) {
    drawLCD(now);
    drawLEDs(now);
    lastDraw = now;
  }

  if (crsf_)
    crsf_->update();
}

void Controller::drawLEDs(const uint32_t now) {
  auto aidx = peerMgr_.getActiveIdx();
  auto active = peerMgr_.findPeer(aidx);
  float energy = active? fabsf(active->lastCmd_.fwd) + fabsf(active->lastCmd_.side) + fabsf(active->lastCmd_.yaw) : 0.0f;
  energy = constrain(energy / 1.5f, 0.0f, 1.0f);
  portENTER_CRITICAL(&ctrlState_.lock);
  float ax = ctrlState_.accelX;
  float ay = ctrlState_.accelY;
  float az = ctrlState_.accelZ;
  portEXIT_CRITICAL(&ctrlState_.lock);
  leds_.update(now, ax, ay, az);

  // Simple behavior → color+fire mapping
  switch (behaviors_.activeIdx_) {
    case 0: leds_.set(CRGB::Cyan, CRGB::Green4, leds::EyeShape::Circle); break; // Happy
    case 1: leds_.set(CRGB::Yellow, CRGB::Orange3, leds::EyeShape::Heart); break; // Excited
    case 2: leds_.set(CRGB::Red, CRGB::Yellow2, leds::EyeShape::Circle);  break; // Scared
    case 3: leds_.set(CRGB::Purple, CRGB::Blue3, leds::EyeShape::Heart); break; // Drunk
    default: leds_.set(CRGB(0x2b65c9), CRGB(0xD02090), leds::EyeShape::Circle); // #2b65c9 #D02090
  }
}

void Controller::drawLCD(const uint32_t now) {
  display_.startFrame();
  auto bgRainbow = display_.timeRainbow(now);
  auto fg = BLACK;
  auto pageBG = BLACK;
  auto validCount = getValidDriveCount();
  auto linkCount = peerMgr_.getRecentCount(now, 1000);
  String title = dynamics_? dynamics_->getStatus() : "?";
  String err;
  if (telem_.vbus > 0.1 && telem_.vbus < lowVoltageCutoff_) err = "low batt!";
  else if (!validCount) err = "no drives!";
  else if (!linkCount) err = "NO LINK";
  if (!err.isEmpty()) {
    title = err;
    fg = WHITE;
    pageBG = SUPERDARKRED;
    bgRainbow = RED;
  }
  if (linkCount > 1) title = str("%d links", linkCount);

  display_.setFont(&FreeSansBold12pt7b);
  display_.drawBorder(bgRainbow);
  display_.drawTitle(title.c_str(), fg, bgRainbow);
  display_.clearContent(pageBG, now);
  display_.drawTelem(telem_, now, pageBG);
  display_.setFont(&FreeSansBold9pt7b);
  if (behaviors_.isActive())
    display_.drawCentered(behaviors_.getName(), pageBG);
  display_.drawVersion(version_, pageBG);
  display_.endFrame();
}
