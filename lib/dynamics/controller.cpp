#include "controller.h"
#include "motiontask.h"
#include <log.h>
#include <NowPacket.h>
#include <dynamics_base.h>
#include <AlfredoCRSF.h>
#include "utils.h"
#include <Arduino.h>
#ifdef IS_M5
#include <M5Unified.h>
#endif
#include <esp_now.h>
#include <WiFi.h>

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
  if (adjustablesCount_ >= MAX_ADJUSTABLES) {
    D_LOG("No space for new adjustable");
    return;
  }
  adjustables_[adjustablesCount_].v = adjustable;
  adjustables_[adjustablesCount_].name = name;
  ++adjustablesCount_;
}

void Controller::setup(SharedState* shared, AlfredoCRSF* crsf) {
  controller_ = this;
  sharedState_ = shared;
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

  if (canPrint())
    D_LOG("finished setup");

  delay(100);
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
  peerMgr_.activate(PEERS_MAX); //changes active peer, propagates to MotionTask
  sendInfoStr(reason.isEmpty()? "disable" : reason);
}

uint32_t lastDraw = 0;
uint32_t lastTxStats = 0;
uint32_t lastUpdateTx = 1000;

void Controller::loop() {
  uint32_t now = millis();

  if (((now - lastTxStats) > 100)) {
    auto statecpy = sharedState_->getCopy();
    auto telem = statecpy.telem;
    if (crsf_ && crsf_->isLinkUp()) {
      crsf_sensor_battery_t crsfBatt = {
        .voltage = htobe16((uint16_t)(telem.vbus * 10.0)),
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

    if (telem.vbus < (lowVoltageCutoff_ - 0.2) && enabled_) {
      disable("low v"); //disable if voltage is too low
    }

    // broadcast telemetry
    broadcast(Cmds::Telemetry, (uint8_t*)&telem, sizeof(Telem));

    lastTxStats = now;
  }

  #ifdef IS_M5
  M5.update(); //updates buttons, etc
  if (M5.BtnA.wasSingleClicked()) {
    if (enabled_) behaviors_.increment();
    else if (selectedTune_ < adjustablesCount_) adjustModeBump(true, true); //clear
    else sendInfoStr("poke");
  } else if (M5.BtnA.wasDoubleClicked()) {
    if (behaviors_.isActive()) behaviors_.clear();
    else adjustModeBump();
  }
  #endif
  float* tunable = selectedTune_ < adjustablesCount_? adjustables_[selectedTune_].v : NULL;

  if ((now - lastUpdateTx) > IMU_UPDATE_PERIOD) {
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

    if (crsfCtrl.timestamp) { //crsf control has extra features
      bool adjBumpMode = selectedTune_ < adjustablesCount_;
      const auto chan8 = crsf_? crsf_->getChannel(8) : 0;
      if (chan8 > 1900) { //held rtl btn
        if (adjBumpMode && tunable) {
          *tunable = pow(10, mapfloat(crsfCtrl.adjust, 0, 1, -2, 2));
          if (crsfCtrl.adjust < 0.01) *tunable = 0; //allow 0 values
        } else if (lastchan8_ < 1900) {
          behaviors_.clearOrRestore();
        }
      } else if (chan8 > 1550 && lastchan8_ < 1550) { //bump up
        if (adjBumpMode) adjustModeBump();
        else behaviors_.increment();
      } else if (chan8 < 1450 && lastchan8_ > 1450) { //bump down
        if (adjBumpMode) adjustModeBump(false);
        else behaviors_.increment(false);
      }
      lastchan8_ = chan8;
    }
    arm = active && (active->lastCmd_.state > 0);
    if ((arm != enabled_)) {
      // -- ENABLING, arming, whatever -- //
      display_.requestRedraw();
    }
    enabled_ = arm;

    //calculate behavior motion
    auto control = behaviors_.iterate(now, (active? active->lastCmd_ : MotionControl()), sharedState_->isBalancing);

    portENTER_CRITICAL(&sharedState_->lock);
    sharedState_->activeCmd = control; //write for the controll thread to read
    sharedState_->crsfActive = isCrsfActive();
    for (uint8_t c = 0; c < CRSF_CHANS; c++)
      sharedState_->crsfChans[c] = crsf_->getChannel(c);
    auto rot = sharedState_->dispRotate;
    portEXIT_CRITICAL(&sharedState_->lock);
    display_.setRotation(rot);

    lastUpdateTx = now;
  } //IMU timing

  if ((now - lastDraw) > 60 || display_.isRedrawRequired()) {
    yield();
    drawLCD(now);
    yield();
    drawLEDs(now);
    yield();
    lastDraw = now;
  }

  if (crsf_)
    crsf_->update();
}

void Controller::adjustModeBump(bool up, bool clear) {
  selectedTune_ = clear? MAX_ADJUSTABLES : ((selectedTune_ + (up? 1 : -1)) % adjustablesCount_);
  if (selectedTune_ < MAX_ADJUSTABLES) {
    const auto& a = adjustables_[selectedTune_];
    sendInfoStr("adj:" + a.name);
    D_LOG("selected adjustable %d/%d: %s %0.01f: # %p", selectedTune_, adjustablesCount_, a.name, a.v? *a.v : -1, a.v);
  } else {
    sendInfoStr("adj-none");
    D_LOG("clear adjustable %d/%d", selectedTune_, adjustablesCount_);
  }
  display_.requestRedraw();
}

void Controller::drawLEDs(const uint32_t now) {
  auto aidx = peerMgr_.getActiveIdx();
  auto active = peerMgr_.findPeer(aidx);
  portENTER_CRITICAL(&sharedState_->lock);
  float ax = sharedState_->accelX;
  float ay = sharedState_->accelY;
  float az = sharedState_->accelZ;
  portEXIT_CRITICAL(&sharedState_->lock);
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
  auto state = sharedState_->getCopy();
  display_.startFrame();
  auto bgRainbow = display_.timeRainbow(now);
  auto fg = BLACK;
  auto pageBG = BLACK;
  auto validCount = state.validDriveCount;
  auto linkCount = peerMgr_.getRecentCount(now, 1000);
  Adj const* adj = (selectedTune_ < MAX_ADJUSTABLES)? &adjustables_[selectedTune_] : nullptr;
  String title = state.title;
  String err;
  if (state.telem.vbus > 0.1 && state.telem.vbus < lowVoltageCutoff_) err = "low batt!";
  else if (!validCount) err = "no drives!";
  else if (!linkCount) err = "NO LINK";
  if (!err.isEmpty()) {
    title = err;
    fg = WHITE;
    pageBG = SUPERDARKRED;
    bgRainbow = RED;
  } else if (adj) {
    title = adj->name;
  } else if (linkCount > 1){
    title = str("%d links", linkCount);
  }

  display_.setFont(&FreeSansBold12pt7b);
  display_.drawBorder(bgRainbow);
  display_.drawTitle(title.c_str(), fg, bgRainbow);
  display_.clearContent(pageBG, now);
  display_.drawTelem(state.telem, now, pageBG);
  if (adj) {
    display_.setFont(&FreeMonoBold18pt7b);
    auto v = *(adj->v);
    String vstr = adj->v? String(v, v < 1? 3 : 1) : "??";
    display_.drawCentered(vstr.c_str(), pageBG);
  } else if (behaviors_.isActive()) {
    display_.setFont(&FreeSansBold9pt7b);
    display_.drawCentered(behaviors_.getName(), pageBG);
  } else { //clear this area
    display_.drawCentered("   ", pageBG);
  }
  display_.drawVersion(version_, pageBG);
  display_.endFrame();
}
