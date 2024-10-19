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

static RCRemote* remote_ = nullptr;

void RCRemote::setup() {
  remote_ = this;
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
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    if (remote_)
      remote_->handleRxPacket(data, len);
  });
  esp_now_register_send_cb([](const uint8_t *mac, esp_now_send_status_t status) {
    if (canPrint())
      Serial.printf("send status: %d\n", status);
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


void RCRemote::loop() {
  uint32_t now = millis();

  #ifdef IS_M5
  M5.update(); //updates buttons, etc
  if (M5.BtnA.wasPressed()) {
    armed_ = !armed_;
    redrawLCD_ = true;
  }
  if (M5.BtnB.wasPressed()) {
    pitchRollOutEn_ = !pitchRollOutEn_;
  }
  #endif
  if ((now - lastPoll_) > 50) {
    updateIMU();

    uint16_t x, y;
    joy_.get_joy_adc_16bits_value_xy(&x, &y);
    bool btn = !joy_.get_button_value();
    if (btn != lastBtn_ && btn) {
      armed_ = !armed_;
    }
    lastBtn_ = btn;
    lastMotion_.state = armed_? 1 : 0;
    lastMotion_.yaw   =  (x - 30300) / 65500.0 * 2;
    lastMotion_.fwd   = -(y - 32600) / 65500.0 * 2;
    lastMotion_.pitch = pitchRollOutEn_? imuFilt_.getPitchDegree() : 0;
    lastMotion_.roll  = pitchRollOutEn_? imuFilt_.getRollDegree() : 0;
    lastMotion_.timestamp = now;
    // TODO deadband, expo.

    uint8_t data[1 + sizeof(MotionControl)] = {0};
    data[0] = (uint8_t) Cmds::MotionControl;
    memcpy(data + 1, &lastMotion_, sizeof(MotionControl));
    auto result = esp_now_send(broadcastAddress, data, sizeof(data));

    Serial.printf("x: %d, y: %d -> fwd: %.2f, yaw: %.2f btn: %d tx %d\n",
      x, y, lastMotion_.fwd, lastMotion_.yaw, btn, result);

    //show red when armed, dark purple when not
    joy_.set_rgb_color(armed_? 0xFF0000 : 0x100010);
    lastPoll_ = now;
  }

  if ((now - lastDraw_) > 60 || redrawLCD_) {
    drawLCD(now);
    lastDraw_ = now;
  }

}

bool RCRemote::updateIMU() {
#ifdef IS_M5
  auto res = M5.Imu.isEnabled()? M5.Imu.update() : 0;
  if (!res) return false;
  auto data = M5.Imu.getImuData(); //no mag data it seems, sadly
  imuFilt_.updateIMU<0,'D'>(data.gyro.y * 2.0, data.gyro.x * 2.0, -data.gyro.z, -data.accel.y, -data.accel.x, data.accel.z); //acc x/y are swapped
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

  //draw 2px line down left, right, and bottom of screen
  lcd_->fillRect(0, 0, 2, lcd_->height(), bgRainbow); //vertical left
  lcd_->fillRect(lcd_->width() - 2, 0, 2, lcd_->height(), bgRainbow); //vertical right
  lcd_->fillRect(0, lcd_->height() - 2, lcd_->width(), 2, bgRainbow); //horizontal bottom

  //show title
  lcd_->setCursor(2, 0);
  lcd_->setTextColor(fg, bgRainbow);
  lcd_->setFont(&FreeSansBold18pt7b);
  drawCentered(title.c_str(), lcd_, bgRainbow);
  lcd_->setCursor(2, lcd_->getCursorY()); //indent-in 2px

  if (redrawLCD_ || (now - lastClear_) > 5000) {
    auto x = lcd_->getCursorX(), y = lcd_->getCursorY();
    lcd_->fillRect(x, y, lcd_->width() - x - 2, lcd_->height() - y - 2, pageBG);
    lastClear_ = now;
    redrawLCD_ = false;
  }

  //draw lastMotion_
  lcd_->setTextColor(fg, bgRainbow);
  lcd_->setFont(&FreeMono12pt7b);
  drawCentered(("f" + String(lastMotion_.fwd)).c_str(), lcd_, bgRainbow);
  drawCentered(("y" + String(lastMotion_.yaw)).c_str(), lcd_, bgRainbow);
  drawCentered(("p" + String(lastMotion_.pitch)).c_str(), lcd_, bgRainbow);
  drawCentered(("r" + String(lastMotion_.roll)).c_str(), lcd_, bgRainbow);

  //bottom aligned
  M5.Lcd.setFont(&Font0);
  M5.Lcd.setCursor(2, M5.Lcd.height() - 9);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf(" %.2f ", 1 / 1000.0 * now);

  // if this has a battery, show that next up from the bottom
  auto power = M5.Power.getType();
  if (power != M5.Power.pmic_unknown) {
    lcd_->setTextColor(WHITE, BLACK);
    lcd_->setFont(&FreeMono12pt7b);
    lcd_->setCursor(2, lcd_->getCursorY() - lcd_->fontHeight() - 2);
    String pwr = String(M5.Power.getBatteryVoltage() / 1000.0) + "V";
    drawCentered(pwr.c_str(), lcd_, BLACK);
  }

  M5.Lcd.endWrite();
}

