#include "rcremote.h"
#include "utils.h"
#include <log.h>
#include <Arduino.h>
#include <NowPacket.h>
#include <esp_task_wdt.h>
#ifdef IS_M5
#include <M5Unified.h>
#endif
#include <WiFi.h>

constexpr uint8_t PKT_SUBT = (uint8_t) Subt::Controller;

#ifdef ESP32S3
constexpr int I2S_SDA = 9;
constexpr int I2S_SCL = 10;
#else
constexpr int I2S_SDA = 32;
constexpr int I2S_SCL = 33;
#endif

// -------------------- //
// ---- Controller ---- //
// -------------------- //

RCRemote::~RCRemote() { }

RCRemote::RCRemote(String version) :
  version_(version) {
}

void RCRemote::setup() {
#ifdef CONFIG_IDF_TARGET_ESP32
  Serial.begin(115200);
#endif

  setCpuFrequencyMhz(80); //lower frequency, better battery

  Serial.begin(115200);
  Serial.setTimeout(10); //very fast, need to keep the ctrl loop running
  D_LOG("RCRemote setup");
  Serial.flush();

  D_LOG("setting up wifi");
  WiFi.mode(WIFI_STA);
  auto res = esp_now_init();
  if (res != ESP_OK)
    D_LOG("ESP-NOW init failed: %d", res);
  static auto remote_ = this;
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    remote_->handleRxPacket(mac, data, len);
  });
  esp_now_register_send_cb([](const uint8_t *mac, esp_now_send_status_t status) {
    constexpr float alpha = 0.1;
    float failv = (status == ESP_NOW_SEND_FAIL)? 1.0f : 0.0f;
    if (status == ESP_NOW_SEND_FAIL) remote_->sendFails_++;
    remote_->lastSentFailFilt_ = alpha * failv + (1.0f - alpha) * remote_->lastSentFailFilt_;
  });
  espnowRegisterMac(BROADCAST_ADDRESS);

#ifdef IS_M5
  D_LOG("setting up m5");
  M5.begin();
  //check if has lcd
  if (M5.Lcd.width() > 0) {
    display_.setLCD(&M5.Lcd);
  }
  M5.Power.begin();
#endif

  D_LOG("finished setup");

  delay(100);
  D_LOG("Ready. Version %s", version_.c_str());
  setupJoystick(2);
}


void RCRemote::handleRxPacket(const uint8_t* mac, const uint8_t* inbuf, uint8_t inlen) {
  comms::NowPacket pkt;
  if (!pkt.parse(inbuf, inlen)) {
    D_LOG("unknown pkt data len %d:", inlen);
    return printBuf(inbuf, inlen);
  }

  auto cmd = (Cmds)pkt.type;
  TPeer* peer = nullptr;
  if ((Subt)pkt.subt == Subt::Robot) { //from a rover, not another remote
    auto peeridx = peerMgr_.findOrMakePeer(mac, true, true, true);
    peer = peerMgr_.findPeer(peeridx);
    if (!peer) { D_LOG("handleRx no peer?!"); }
  }
  //now actually handle packets
  if (cmd == Cmds::Telemetry && pkt.payloadLen == sizeof(Telem)) {
    auto telem = *((const Telem*) pkt.payload);
    telem.timestamp = millis();
    if (peer) peer->lastCmd_ =telem; //save
  } else if (cmd == Cmds::PingReply) {
    D_LOG("Ping reply from %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else if (cmd == Cmds::InfoStr && peer) {
    infoDisp_ = String((char*)pkt.payload, pkt.payloadLen);
    D_LOG("info: %s", infoDisp_.c_str());
  }
}

uint8_t RCRemote::nextPeer(uint8_t old, bool allowold) {
  for (uint8_t i = 0; i < PEERS_MAX; i++) {
    auto& peer = peerMgr_.peers_[i];
    if (i == old) continue;
    if (peer.isValid())
      return i;
  }
  return allowold? old : PEERS_MAX;
}

uint8_t RCRemote::validPeerCount() const {
  uint8_t ret = 0;
  for (uint8_t i = 0; i < PEERS_MAX; i++)
    if (peerMgr_.peers_[i].isValid())
      ret++;
  return ret;
}

void RCRemote::setArmState(bool arm) {
  if (powerSaveMode_)
    return;
  if (arm) { //
    for (float f : {lastMotion_.fwd, lastMotion_.side, lastMotion_.yaw}) {
      if (abs(f) > 0.05) {
        infoDisp_ = "stick err";
        return;
      }
    }
  }
  armed_ = arm;
}
// [dug] New peer dc:54:75:cb:ba:d0 -> 0x0

void RCRemote::send(Cmds cmd, const uint8_t* pyld, uint8_t len) {
  uint8_t txBuf[comms::HEADER_OVERHEAD + len] = {0};
  uint8_t txLen = comms::NowPacket::serialise((uint8_t)cmd, PKT_SUBT, pyld, len, txBuf, sizeof(txBuf));
  auto peer = peerMgr_.findPeer(peerMgr_.activePeer_);
  if (peer && txLen) {
    auto result = esp_now_send(peer->mac, txBuf, txLen);
  } else D_LOG("tx can't send %p %d", peer, txLen);
}
void RCRemote::broadcast(Cmds cmd, const uint8_t* pyld, uint8_t len) {
  uint8_t txBuf[comms::HEADER_OVERHEAD + len] = {0};
  uint8_t txLen = comms::NowPacket::serialise((uint8_t)cmd, PKT_SUBT, pyld, len, txBuf, sizeof(txBuf));
  if (txLen == 0) { D_LOG("txlen 0"); return; }
  auto result = esp_now_send(BROADCAST_ADDRESS, txBuf, txLen);
}

void RCRemote::setupJoystick(uint16_t maxtries) {
  if (joystickSetup_) return;
  D_LOG("setting up joystick");
  uint16_t fails = 0;
  while (!joy_.begin(&Wire, JOYSTICK2_ADDR, I2S_SDA, I2S_SCL)) {
    D_LOG("JoyC init failed: %d", fails++);
    if (fails > maxtries) return;
    delay(10);
  }
  joy_.get_joy_adc_16bits_value_xy(&joyCenterX_, &joyCenterY_);
  D_LOG("joystick center %d %d", joyCenterX_, joyCenterY_);
  joystickSetup_ = true;
}

void RCRemote::pollJoystick(uint32_t now) {
  if (!joystickSetup_) return;
  uint16_t x = 0, y = 0;
  joy_.get_joy_adc_16bits_value_xy(&x, &y);
  if (x == 0 && y == 0) { D_LOG("dis val"); return; } //no reading
  if (x == 257 && y == 257) { D_LOG("dis val2"); return; } //error reading
  bool joybtn = !joy_.get_button_value();
  uint32_t dbtnt = (now - lastJoyBtnChange_);
  if (lastJoyBtn_ != joybtn) {
    lastWasMoved_ = now;
    if (!joybtn && dbtnt < BTN_SHORTPRESS_MAX) { //short press
      if (powerSaveMode_) {
        //nothing! wake up
      } else if (armed_) {
        send(Cmds::ModeChange);
      } else {
        setArmState(true);
      }
    }
    lastJoyBtnChange_ = now;
  }
  lastJoyBtn_ = joybtn;
  MotionControl mc;
  mc.state = armed_? 1 : 0;
  mc.adjust = lastMotion_.adjust;
  float deadband_ = 0.05;
  float expo_ = 1.5;
  mc.yaw   = expo(deadband( (x - joyCenterX_) / 65500.0 * 2, deadband_), expo_);
  mc.fwd   = expo(deadband(-(y - joyCenterY_) / 65500.0 * 2, deadband_), expo_);
  if (joybtn) { //when joystick is held down, control side
    mc.side = mc.yaw;
    mc.yaw = 0;
  }
  mc.side = constrain(mc.side, -1.0, 1.0);
  mc.timestamp = now;
  //check for major discontinuity
  if (abs(mc.yaw - lastMotion_.yaw) > 1.0 || abs(mc.fwd - lastMotion_.fwd) > 1.0) {
    D_LOG("Discontinuity: [%d,%d]", x, y);
  } else {
    //apply smoothing
    float alpha = 0.3f; //0.3 is good for 25Hz, 0.2 for 50Hz
    mc.fwd = alpha * lastMotion_.fwd + (1.0f - alpha) * mc.fwd;
    mc.yaw = alpha * lastMotion_.yaw + (1.0f - alpha) * mc.yaw;
    mc.side = alpha * lastMotion_.side + (1.0f - alpha) * mc.side;

    //send it to selected rover
    send(Cmds::MotionControl, (uint8_t*)&mc, sizeof(MotionControl));
  }
  lastMotion_ = mc;

  D_LOG("xy: [%d,%d] -> f,y,s: [%4.2f,%4.2f,%4.2f] -> %0.1f %d", x, y, lastMotion_.fwd, lastMotion_.yaw, lastMotion_.side, 100.0f * lastSentFailFilt_, sendFails_);

  //show red when armed, dark purple when not
  joy_.set_rgb_color(armed_? 0xFF0000 : 0x100010);
  if (abs(lastMotion_.fwd) > 0.1 || abs(lastMotion_.yaw) > 0.1)
    lastWasMoved_ = now;
}

void RCRemote::setWakeupPower(bool wakeup) {
  powerSaveMode_ = !wakeup;
  M5.Lcd.setBrightness(wakeup? 200 : 0);
  // auto pwr = M5.Power.M5pm1;
  // auto g2m = pwr.setGPIOMode(m5::M5PM1_Class::gpio2, m5::M5PM1_Class::output);
  // auto g2  = pwr.setGPIOOutput(m5::M5PM1_Class::gpio2, wakeup);
  // D_LOG("wakeup[%d], g2 %d %d", wakeup, g2m, g2);
  if (!armed_ && joystickSetup_ && powerSaveMode_)
    joy_.set_rgb_color(0); //power off led
}

void RCRemote::loop() {
  uint32_t now = millis();

  #ifdef IS_M5
  M5.update(); //updates buttons, etc
  if (M5.BtnA.wasPressed()) {
    lastWasMoved_ = now;
    setArmState(!armed_);
    display_.requestRedraw();
  }
  if (M5.BtnB.wasDecideClickCount()) { //left side button released
    lastWasMoved_ = now;
    bool single = (M5.BtnB.wasSingleClicked());
    if (armed_) lastMotion_.adjust = fmod(lastMotion_.adjust + (single? 0.2f : -0.2), 1.0f); //bump up/down
    else peerMgr_.activePeer_ = nextPeer(peerMgr_.activePeer_, true); //change peer
    display_.requestRedraw();
  }
  if (M5.BtnPWR.wasDecideClickCount()) { //right side button on older targets
    if (armed_) setArmState(false);
    else M5.Power.powerOff();
  }
  #endif

  // Send ping every 500ms
  if ((now - lastPing_) > 500) {
    broadcast(Cmds::Ping);
    lastPing_ = now;
  }

  if ((now - lastPoll_) > ((!armed_ || powerSaveMode_)? 200 : 25)) {

    if (joystickSetup_) {
      pollJoystick(now);
    } else {
      setupJoystick();
    }
    lastPoll_ = now;
  }

  if ((now - lastDraw_) > 60 || display_.isRedrawRequired()) {

    //filter isCharging because it's flakey
    float ischg = M5.Power.isCharging()? 1.0f : 0.0f;
    constexpr float alpha = 0.01;
    isChargingFilt_ = alpha * ischg + (1.0f - alpha) * isChargingFilt_;
    isCharging_ = isChargingFilt_ > 0.2f;

    if (!powerSaveMode_) { drawLCD(now); }
    lastDraw_ = now;
  }

  // -- power handling -- //

  #if ARDUINO_USB_CDC_ON_BOOT
  bool disableLightSleep = Serial.isConnected() || isCharging_; //don't sleep when plugged in!
  #else
  bool disableLightSleep = false;
  #endif

  uint32_t sinceMoved = now - lastWasMoved_;
  if (!powerSaveMode_ && sinceMoved > DISPLAY_SLEEP_MS) {
    setWakeupPower(false); //turn of LCD
  } else if (powerSaveMode_ && sinceMoved < DISPLAY_SLEEP_MS) {
    setWakeupPower(true); //turn everything back on
  } else if (!armed_ && powerSaveMode_ && (sinceMoved > IDLE_POWEROFF_SLEEP_MS) && !disableLightSleep) {
    D_LOG("power off after %dms", sinceMoved);
    Serial.flush();
    delay(100);
    M5.Power.powerOff();
  }
  if (powerSaveMode_ && !disableLightSleep) {
    //ESP32 light sleep
    D_LOG("light sleep after %dms", sinceMoved);
    Serial.flush();
    M5.Power.lightSleep(100 * 1000); //100ms
    delay(2);
    yield();
  }
}

void RCRemote::drawLCD(const uint32_t now) {
  auto lcd = display_.getLCD();
  if (!lcd) return;
  auto peer = peerMgr_.findPeer(peerMgr_.activePeer_);
  auto validcount = validPeerCount();

  display_.startFrame();
  auto bgRainbow = display_.timeRainbow(now);
  auto fg = BLACK;
  auto pageBG = BLACK;

  String title = armed_? "GO" : "--";
  if (validcount > 0) {
    title += " R" + String(peerMgr_.activePeer_ + 1);
  }
  if (lastSentFailFilt_ > 0.5) {
    title = "no link";
    fg = RED;
    bgRainbow = SUPERDARKRED;
  }

  display_.drawBorder(bgRainbow);
  display_.drawTitle(title, fg, bgRainbow);
  display_.clearContent(pageBG, now);

  //draw lastMotion_
  lcd->setTextColor(bgRainbow, pageBG);
  lcd->setFont(&FreeMono12pt7b);
  if (joystickSetup_) {
    display_.drawCentered(("f" + String(lastMotion_.fwd  )).c_str(), pageBG);
    display_.drawCentered(("y" + String(lastMotion_.yaw  )).c_str(), pageBG);
  } else { display_.drawCentered("no joy", pageBG); }

  // Draw telemetry using the common function
  display_.drawTelem(peer->lastCmd_, now, pageBG);

  if (true) { //validcount > 1) { //only show when handling multiple bots
    lcd->setFont(&FreeMono12pt7b);
    lcd->setTextColor(bgRainbow, pageBG);
    String roverInfo =  " #" + String(peerMgr_.activePeer_ + 1) + "/" + String(validcount);
    display_.drawCentered(roverInfo.c_str(), pageBG);
  }
  if (!infoDisp_.isEmpty()) {
    lcd->setFont(&FreeMono12pt7b);
    lcd->setTextColor(WHITE, pageBG);
    display_.drawCentered(infoDisp_.c_str(), pageBG);
  }

  display_.drawVersion(version_, pageBG);

  if (M5.Power.getType() != M5.Power.pmic_unknown) { //internal battery
    lcd->setTextColor(WHITE, pageBG);
    lcd->setFont(&FreeMono12pt7b);
    lcd->setCursor(0, lcd->height() - 2 * lcd->fontHeight());
    String pwr = String(M5.Power.getBatteryVoltage() / 1000.0) + "V";
    if (isCharging_) pwr += "c";
    display_.drawCentered(pwr.c_str(), pageBG);
  }

  display_.endFrame();
}

