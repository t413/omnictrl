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
    lcd_ = &M5.Lcd;
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
    redrawLCD_ = true;
  }
  if (M5.BtnB.wasPressed()) {
    lastWasMoved_ = now;
    pitchRollOutEn_ = !pitchRollOutEn_;
    if (pitchRollOutEn_)
      imuFilt_ = Madgwick(0.2, 50); //resets
  }
  #endif

  if ((now - lastPoll_) > ((armed_ || !powerSaveMode_)? 50 : 200)) {
    updateIMU();

    uint16_t x, y;
    joy_.get_joy_adc_16bits_value_xy(&x, &y);
    bool btn = !joy_.get_button_value();
    if (btn != lastBtn_ && btn) {
      setArmState(!armed_);
    }
    lastBtn_ = btn;
    lastMotion_.state = armed_? 1 : 0;
    lastMotion_.yaw   =  (x - 30300) / 65500.0 * 2;
    lastMotion_.fwd   = -(y - 32600) / 65500.0 * 2;
    lastMotion_.pitch = pitchRollOutEn_? imuFilt_.getPitchDegree() / 2.0 : 0;
    lastMotion_.roll  = pitchRollOutEn_? imuFilt_.getRollDegree()  / 2.0 : 0;
    lastMotion_.timestamp = now;
    // TODO deadband, expo.

    uint8_t data[1 + sizeof(MotionControl)] = {0};
    data[0] = (uint8_t) Cmds::MotionControl;
    memcpy(data + 1, &lastMotion_, sizeof(MotionControl));
    auto result = esp_now_send(broadcastAddress, data, sizeof(data));

    if (canPrint()) {
      Serial.printf("xy: [%d,%d] -> f,y,p,r: [%.2f,%.2f,\t%.2f,%.2f] -> %s\n",
        x, y, lastMotion_.fwd, lastMotion_.yaw, lastMotion_.pitch, lastMotion_.roll,
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
  if ((now - lastDraw_) > 60 || redrawLCD_) {
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

uint16_t rainbowColor(float v);
const uint16_t SUPERDARKRED = lgfx::color565(50, 0, 0);
const uint16_t SUPERDARKBLUE = lgfx::color565(0, 0, 50);

void RCRemote::drawLCD(const uint32_t now) {
  lcd_->startWrite();
  auto bgRainbow = rainbowColor((now >> 2) % 1000 / 1000.0);
  auto fg = BLACK;
  auto pageBG = BLACK;

  String title = armed_? "GO" : "--";
  if (lastSentFail_) {
    title = "no link";
    fg = RED;
    bgRainbow = SUPERDARKRED;
  }

  //draw 2px line down left, right, and bottom of screen
  uint16_t borderW = 2;
  lcd_->fillRect(0, 0, borderW, lcd_->height(), bgRainbow); //vertical left
  lcd_->fillRect(lcd_->width() - borderW, 0, borderW, lcd_->height(), bgRainbow); //vertical right
  lcd_->fillRect(0, lcd_->height() - borderW, lcd_->width(), borderW, bgRainbow); //horizontal bottom

  //show title
  lcd_->setCursor(borderW, 0);
  lcd_->setTextColor(fg, bgRainbow);
  lcd_->setFont(&FreeSansBold18pt7b);
  drawCentered(title.c_str(), lcd_, bgRainbow, borderW);
  lcd_->setCursor(borderW, lcd_->getCursorY()); //indent-in 2px

  if (redrawLCD_ || (now - lastClear_) > 5000) {
    auto x = lcd_->getCursorX(), y = lcd_->getCursorY();
    lcd_->fillRect(x, y, lcd_->width() - x - borderW, lcd_->height() - y - borderW, pageBG);
    lastClear_ = now;
    redrawLCD_ = false;
  }

  //draw lastMotion_
  lcd_->setTextColor(bgRainbow, pageBG);
  lcd_->setFont(&FreeMono12pt7b);
  drawCentered(("f" + String(lastMotion_.fwd  )).c_str(), lcd_, pageBG, borderW);
  drawCentered(("y" + String(lastMotion_.yaw  )).c_str(), lcd_, pageBG, borderW);
  drawCentered(("p" + String(lastMotion_.pitch)).c_str(), lcd_, pageBG, borderW);
  drawCentered(("r" + String(lastMotion_.roll )).c_str(), lcd_, pageBG, borderW);

  //bottom aligned
  M5.Lcd.setFont(&Font0);
  M5.Lcd.setCursor(borderW, M5.Lcd.height() - borderW - M5.Lcd.fontHeight());
  auto bttmY = M5.Lcd.getCursorY();
  M5.Lcd.setTextColor(WHITE, BLACK);
  drawCentered(version_.c_str(), lcd_, pageBG, borderW);

  // if this has a battery, show that next up from the bottom
  auto power = M5.Power.getType();
  if (power != M5.Power.pmic_unknown) {
    lcd_->setTextColor(WHITE, pageBG);
    lcd_->setFont(&FreeMono12pt7b);
    lcd_->setCursor(borderW, bttmY - lcd_->fontHeight());
    String pwr = String(M5.Power.getBatteryVoltage() / 1000.0) + "V";
    drawCentered(pwr.c_str(), lcd_, pageBG, borderW);
  }

  M5.Lcd.endWrite();
}

