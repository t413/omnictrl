#include "rcremote.h"
#include "utils.h"
#include <Arduino.h>
#ifdef IS_M5
#include <M5Unified.h>
#endif
#include <WiFi.h>

#define POS_X 0
#define POS_Y 1

bool canPrint();

uint8_t broadcastAddress[] = {0xDC, 0x54, 0x75, 0xCB, 0xBA, 0xD0};

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
  if (canPrint()) { //only use the virtual serial port if it's available
    Serial.println("RCRemote setup");
    Serial.flush();
  }
  delay(100);

  auto jres = joy_.begin(&Wire, JOYSTICK2_ADDR, 32, 33);
  if (!jres && canPrint())
    Serial.printf("JoyC init failed: %d\n", jres);

  WiFi.mode(WIFI_STA);
  auto res = esp_now_init();
  if (res != ESP_OK && canPrint())
    Serial.printf("ESP-NOW init failed: %d\n", res);
  static auto remote_ = this;
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    remote_->handleRxPacket(data, len);
  });
  esp_now_register_send_cb([](const uint8_t *mac, esp_now_send_status_t status) {
    remote_->lastSentFail_ = (status == ESP_NOW_SEND_FAIL);
  });
  // Register peer
  memcpy(peerInfo_.peer_addr, broadcastAddress, 6);
  peerInfo_.channel = 0;
  peerInfo_.encrypt = false;
  res = esp_now_add_peer(&peerInfo_);
  if (res != ESP_OK && canPrint())
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

  if (canPrint())
    Serial.println("finished setup");

  delay(100);
  Serial.printf("Ready. Version %s\n", version_.c_str());
}


void RCRemote::handleRxPacket(const uint8_t* buf, uint8_t len) {
  if (canPrint()) {
    Serial.printf("RX: {");
    for (int i = 0; i < len; i++)
      Serial.printf("0x%02x, ", buf[i]);
    Serial.println("}");
  }
  auto cmd = len >= 1? (Cmds) buf[0] : Cmds::None;
  if (cmd == Cmds::Telemetry && len == (1 + sizeof(Telem))) {
    lastTelemetry_ = *((const Telem*) (buf + 1));
    lastTelemetry_.timestamp = millis();
  }
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
  #endif

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

      //send it
      uint8_t data[1 + sizeof(MotionControl)] = {0};
      data[0] = (uint8_t) Cmds::MotionControl;
      memcpy(data + 1, &mc, sizeof(MotionControl));
      auto result = esp_now_send(broadcastAddress, data, sizeof(data));
    }
    lastMotion_ = mc;

    if (canPrint()) {
      Serial.printf("xy: [%d,%d] -> f,y,p,r: [%.2f,%.2f,\t%.2f] -> %s\n",
        x, y, lastMotion_.fwd, lastMotion_.yaw, lastMotion_.side,
        lastSentFail_? "fail" : "ok"
      );
    }

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

