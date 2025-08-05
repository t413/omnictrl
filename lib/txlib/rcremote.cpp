#include "rcremote.h"
#include "utils.h"
#include <Arduino.h>
#ifdef IS_M5
#include <M5Unified.h>
#endif
#include <WiFi.h>

#define POS_X 0
#define POS_Y 1

const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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

  Serial.begin(115200);
  Serial.setTimeout(10); //very fast, need to keep the ctrl loop running
  Serial.println("RCRemote setup");
  Serial.flush();
  delay(100);

  auto jres = joy_.begin(&Wire, JOYSTICK2_ADDR, 32, 33);
  if (!jres)
    Serial.printf("JoyC init failed: %d\n", jres);

  WiFi.mode(WIFI_STA);
  auto res = esp_now_init();
  if (res != ESP_OK)
    Serial.printf("ESP-NOW init failed: %d\n", res);
  static auto remote_ = this;
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    remote_->handleRxPacket(mac, data, len);
  });
  esp_now_register_send_cb([](const uint8_t *mac, esp_now_send_status_t status) {
    remote_->lastSentFail_ = (status == ESP_NOW_SEND_FAIL);
  });
  // Register peer
  memcpy(peerInfo_.peer_addr, broadcastAddress, 6);
  peerInfo_.channel = 0;
  peerInfo_.encrypt = false;
  res = esp_now_add_peer(&peerInfo_);
  if (res != ESP_OK)
    Serial.printf("ESP-NOW add peer failed: %d\n", res);


#ifdef IS_M5
  M5.begin();
  //check if has lcd
  if (M5.Lcd.width() > 0) {
    display_.setLCD(&M5.Lcd);
  }
  M5.Power.begin();
  if (M5.Imu.isEnabled()) {
    M5.Imu.loadOffsetFromNVS();
    // M5.Imu.setAxisOrderRightHanded( ??
    imuFilt_.setFrequency(50); //50Hz, 20ms
  }
#endif

  Serial.println("finished setup");

  delay(100);
  Serial.printf("Ready. Version %s\n", version_.c_str());
}


void RCRemote::handleRxPacket(const uint8_t* mac, const uint8_t* buf, uint8_t len) {
  if (Serial && Serial.availableForWrite() > 0) {
    Serial.printf("RX: {");
    for (int i = 0; i < len; i++)
      Serial.printf("0x%02x, ", buf[i]);
    Serial.println("}");
  }
  auto cmd = len >= 1? (Cmds) buf[0] : Cmds::None;
  if (cmd == Cmds::Telemetry && len == (1 + sizeof(Telem))) {
    lastTelemetry_ = *((const Telem*) (buf + 1));
    lastTelemetry_.timestamp = millis();
  } else if (cmd == Cmds::PingReply) {
    Serial.printf("Ping reply from %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  if (findClient(mac) == maxRovers_) {
    Serial.printf("Discovered new rover\n");
    addClient(mac); // Add newly discovered rover!
    setTxDest(getClientDest()); // Update peer info to new target if needed
  }
}

int RCRemote::findClient(const uint8_t* mac) {
  for (int i = 0; i < maxRovers_; i++) {
    if (memcmp(discoveredRovers_[i], mac, 6) == 0) {
      return i;
    }
  }
  return maxRovers_; // Not found
}

void RCRemote::addClient(const uint8_t* mac) {
  const uint8_t idx = roverCount_ % maxRovers_;
  memcpy(discoveredRovers_[idx], mac, 6);
  roverCount_++;
  Serial.printf("Discovered rover %d: %02x:%02x:%02x:%02x:%02x:%02x\n",
    idx, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  display_.requestRedraw();
}

void RCRemote::setTxDest(const uint8_t* mac) {
  // esp_now_del_peer(peerInfo_.peer_addr); // Remove existing peer
  memcpy(peerInfo_.peer_addr, mac, 6); // Set new destination
  auto res = esp_now_add_peer(&peerInfo_);
  if (res != ESP_OK)
    Serial.printf("ESP-NOW switch peer failed: %d\n", res);
  Serial.printf("Switched to rover %d: %02x:%02x:%02x:%02x:%02x:%02x\n", selectedRover_, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

const uint8_t* RCRemote::getClientDest() const {
  return (roverCount_ > 0)? discoveredRovers_[selectedRover_] : broadcastAddress;
}

void RCRemote::setArmState(bool arm) {
  if (powerSaveMode_)
    return;
  if (armed_) { //disarming
    armed_ = false; //always just disarm
  } else if (!armed_ && !lastSentFail_) { //arming check
    armed_ = true;
  }
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
  if (M5.BtnB.wasPressed()) { //left side button, toggle IMU control
    lastWasMoved_ = now;
    pitchRollOutEn_ = !pitchRollOutEn_;
    if (pitchRollOutEn_)
      imuFilt_ = Madgwick(0.2, 50); //resets
    display_.requestRedraw();
  }
  if (M5.BtnPWR.wasPressed()) { //right side button, switch between rovers
    lastWasMoved_ = now;
    selectedRover_ = roverCount_ > 0? (selectedRover_ + 1) % roverCount_ : 0; //increment / wrap
    auto newmac = getClientDest();
    setTxDest(newmac); // Update peer info to new target
    display_.requestRedraw();
  }
  #endif

  // Send ping every 500ms
  if ((now - lastPing_) > 500) {
    uint8_t pingData[1] = {(uint8_t)Cmds::Ping};
    esp_now_send(broadcastAddress, pingData, sizeof(pingData));
    lastPing_ = now;
  }

  if ((now - lastPoll_) > ((armed_ || !powerSaveMode_)? 25 : 200)) {
    updateIMU();

    uint16_t x = 30300, y = 32600;
    joy_.get_joy_adc_16bits_value_xy(&x, &y);
    bool joybtn = !joy_.get_button_value();

    lastBtn_ = joybtn;
    MotionControl mc;
    mc.state = armed_? 1 : 0;
    float deadband_ = 0.05;
    float expo_ = 1.5;
    mc.yaw   = expo(deadband( (x - 30300) / 65500.0 * 2, deadband_), expo_);
    mc.fwd   = expo(deadband(-(y - 32600) / 65500.0 * 2, deadband_), expo_);
    if (pitchRollOutEn_) { //use imu roll for side control
      mc.side = expo(deadband(imuFilt_.getRollDegree(), 10.0) / 20.0, expo_ * 2);
    } else if (joybtn) { //when joystick is held down, control side
      mc.side = mc.yaw;
      mc.yaw = 0;
    }
    mc.side = constrain(mc.side, -1.0, 1.0);
    // mc.roll  = pitchRollOutEn_? expo(deadband(imuFilt_.getRollDegree()  / 2.0, 1.0), expo_) : 0;
    mc.timestamp = now;
    //check for major discontinuity
    if (abs(mc.yaw - lastMotion_.yaw) > 1.0 || abs(mc.fwd - lastMotion_.fwd) > 1.0) {
      Serial.printf("Discontinuity: [%d,%d]\n", x, y);
    } else {
      //apply smoothing
      float alpha = 0.3f; //0.3 is good for 25Hz, 0.2 for 50Hz
      mc.fwd = alpha * lastMotion_.fwd + (1.0f - alpha) * mc.fwd;
      mc.yaw = alpha * lastMotion_.yaw + (1.0f - alpha) * mc.yaw;
      mc.side = alpha * lastMotion_.side + (1.0f - alpha) * mc.side;

      //send it to selected rover
      uint8_t data[1 + sizeof(MotionControl)] = {0};
      data[0] = (uint8_t) Cmds::MotionControl;
      memcpy(data + 1, &mc, sizeof(MotionControl));
      const uint8_t* target = (roverCount_ > 0)? discoveredRovers_[selectedRover_] : broadcastAddress;
      auto result = esp_now_send(target, data, sizeof(data));
    }
    lastMotion_ = mc;

    Serial.printf("xy: [%d,%d] -> f,y,p,r: [%.2f,%.2f,\t%.2f] -> %s\n",
      x, y, lastMotion_.fwd, lastMotion_.yaw, lastMotion_.side,
      lastSentFail_? "fail" : "ok"
    );

    //show red when armed, dark purple when not
    joy_.set_rgb_color(armed_? 0xFF0000 : 0x100010);
    if (armed_ || abs(lastMotion_.fwd) > 0.1 || abs(lastMotion_.yaw) > 0.1)
      lastWasMoved_ = now;
    lastPoll_ = now;
  }

  uint32_t sinceMoved = now - lastWasMoved_;
  if ((now - lastDraw_) > 60 || display_.isRedrawRequired()) {
    if (powerSaveMode_ && sinceMoved < 10000) {
      powerSaveMode_ = false;
      M5.Lcd.setBrightness(200); //turn on LCD

    } else if (!powerSaveMode_ && !armed_ && sinceMoved > 10000) {
      joy_.set_rgb_color(0xFFFF00); //yellow
      //turn of LCD
      powerSaveMode_ = true;
      M5.Lcd.setBrightness(0);
    } else if (!armed_ && powerSaveMode_ && (sinceMoved > 2 * 60 * 1000) && lastSentFail_) { //2 minutes
      M5.Power.powerOff();
    } else {
      drawLCD(now);
    }
    lastDraw_ = now;
  }

}

bool RCRemote::updateIMU() {
#ifdef IS_M5
  auto res = M5.Imu.isEnabled()? M5.Imu.update() : 0;
  if (!res) return false;
  auto data = M5.Imu.getImuData(); //no mag data it seems, sadly
  imuFilt_.updateIMU<0,'D'>(data.gyro.y * 2.0, data.gyro.x * 2.0, -data.gyro.z / 2.0, -data.accel.y, -data.accel.x, data.accel.z); //acc x/y are swapped
#endif
  return true;
}

void RCRemote::drawLCD(const uint32_t now) {
  auto lcd = display_.getLCD();
  if (!lcd) return;

  display_.startFrame();
  auto bgRainbow = display_.timeRainbow(now);
  auto fg = BLACK;
  auto pageBG = BLACK;

  String title = armed_? "GO" : "--";
  if (roverCount_ > 0) {
    title += " R" + String(selectedRover_ + 1);
  }
  if (lastSentFail_) {
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
  display_.drawCentered(("f" + String(lastMotion_.fwd  )).c_str(), pageBG);
  display_.drawCentered(("y" + String(lastMotion_.yaw  )).c_str(), pageBG);
  if (pitchRollOutEn_)
    display_.drawCentered(("s" + String(lastMotion_.side  )).c_str(), pageBG);

  // Draw telemetry using the common function
  if ((now - lastTelemetry_.timestamp) < 1000) {
    display_.drawTelem(lastTelemetry_, now, pageBG);
  } else {
    // Show "no telemetry" if no recent data
    lcd->setFont(&FreeSans18pt7b);
    lcd->setTextColor(RED, pageBG);
    display_.drawCentered("no telem", pageBG);
  }

  if (selectedRover_ < maxRovers_) { //always true (for now)
    lcd->setFont(&FreeMono12pt7b);
    lcd->setTextColor(bgRainbow, pageBG);
    auto mac = getClientDest();
    String roverInfo =  String(mac[5], HEX) + " #" + String(selectedRover_) + "/" + String(roverCount_);
    display_.drawCentered(roverInfo.c_str(), pageBG);
  }

  display_.drawVersion(version_, pageBG);

  if (M5.Power.getType() != M5.Power.pmic_unknown) { //internal battery
    lcd->setTextColor(WHITE, pageBG);
    lcd->setFont(&FreeMono12pt7b);
    lcd->setCursor(0, lcd->height() - 2 * lcd->fontHeight());
    String pwr = String(M5.Power.getBatteryVoltage() / 1000.0) + "V";
    display_.drawCentered(pwr.c_str(), pageBG);
  }

  display_.endFrame();
}

