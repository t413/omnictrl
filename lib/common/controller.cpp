#include "controller.h"
#include "utils.h"
#include <Arduino.h>
#include <cybergear.h>
#include <can_esp32_twai.h>
#ifdef IS_M5
#include <M5Unified.h>
#endif
#include <esp_now.h>
#include <WiFi.h>

#if NUM_DRIVES == 3
const uint8_t DRIVE_IDS[NUM_DRIVES] = {0x7D, 0x7E, 0x7F};
#else
const uint8_t DRIVE_IDS[NUM_DRIVES] = {0};
#endif
#if defined(ARDUINO_M5Stack_ATOMS3)
#define PIN_CRSF_RX 5
#define PIN_CRSF_TX 6
#define PIN_CAN_RX 1
#define PIN_CAN_TX 2
#elif defined(ARDUINO_M5Stick_C)
#define PIN_CRSF_RX 33
#define PIN_CRSF_TX 32
#define PIN_CAN_RX 26
#define PIN_CAN_TX 36
#else
#error "unknown board"
#endif

#define MAX_TILT 20.0
constexpr uint32_t IMU_UPDATE_PERIOD = 20; //ms

bool canPrint() {
  #if ARDUINO_USB_CDC_ON_BOOT
  return Serial && Serial.availableForWrite();
  #else
  return true;
  #endif
}


CanEsp32Twai twaiInterface_;


// -------------------- //
// ---- Controller ---- //
// -------------------- //

Controller::~Controller() { }

Controller::Controller(String version) :
        version_(version) {
  imuFilt_ .setFrequency(1000 / IMU_UPDATE_PERIOD); //50Hz, 20ms
}

static Controller* controller_ = nullptr;

void Controller::setup() {
  controller_ = this;
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

  Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  crsf_.begin(Serial1);

  twaiInterface_.setup(PIN_CAN_RX, PIN_CAN_TX, &Serial);

  for (int i = 0; i < NUM_DRIVES; i++) {
    drives_[i] = new CyberGearDriver(DRIVE_IDS[i], &twaiInterface_);
  }

  for (int i = 0; i < NUM_DRIVES; i++) {
    drives_[i]->setMode(MotorMode::Disabled); // start with disabled mode
    // drives_[i]->driver_.set_position_ref(0.0); // set initial rotor position
  }

  if (canPrint())
    Serial.printf("finished CAN setup\n");

  WiFi.mode(WIFI_STA);
  auto res = esp_now_init();
  if (res != ESP_OK && canPrint())
    Serial.printf("ESP-NOW init failed: %d\n", res);
  esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
    if (controller_)
      controller_->handleRxPacket(data, len);
  });

#ifdef IS_M5
  M5.begin();
  //check if has lcd
  if (M5.Lcd.width() > 0) {
    lcd_ = &M5.Lcd;
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
  return crsf_.isLinkUp() || ((now - lastMotionCmd_.timestamp) < 500);
}

uint8_t Controller::getValidDriveCount() const {
  uint8_t validCount = 0;
  for (int i = 0; i < NUM_DRIVES; i++) {
    bool hasRecent = (millis() - drives_[i]->getLastStatusTime()) < 1000;
    if (hasRecent) validCount++;
  }
  return validCount;
}

void Controller::resetPids() {
  yawCtrl_.reset();
  balanceCtrl_.reset();
  balanceSpeedCtrl_.reset();
  balanceYawCtrl_.reset();
}

void Controller::handleRxPacket(const uint8_t* buf, uint8_t len) {
  if (canPrint()) {
    Serial.printf("RX(%d): [", len);
    for (int i = 0; i < len; i++)
      Serial.printf("0x%02x, ", buf[i]);
    Serial.println("]");
  }
  auto cmd = len >= 1? (Cmds) buf[0] : Cmds::None;
  if (cmd == Cmds::MotionControl && len == (1 + sizeof(MotionControl))) {
    MotionControl rxmc = *((const MotionControl*) (buf + 1));
    rxmc.timestamp = millis();
    if (canPrint())
      Serial.printf("MC: a%d fwd %06.2f yaw %06.2f pitch %06.2f roll %06.2f\n", rxmc.state, rxmc.fwd, rxmc.yaw, rxmc.pitch, rxmc.roll);
    lastMotionCmd_ = rxmc;
  }
}

uint32_t lastDraw = 0;
uint32_t lastDrive = 1000;
uint32_t lastPollStats = 0;
uint32_t lastTxStats = 0;
uint32_t lastClear = 0;

void Controller::loop() {
  uint32_t now = millis();

  if ((now - lastPollStats) > 500) {
    for (int i = 0; i < NUM_DRIVES; i++)
      drives_[i]->requestStatus();
    lastPollStats = now;
    if (canPrint() && !isLinkUp(now))
      Serial.println("MAC: " + WiFi.macAddress());
  }
#if 0
  if (((now - lastTxStats) > 100) && crsf_.isLinkUp()) {
    crsf_sensor_battery_t crsfBatt = {
      .voltage = htobe16((uint16_t)(lastBusStatus_.Bus_Voltage * 10.0)),
      .current = htobe16((uint16_t)(lastBusStatus_.Bus_Current * 10.0)),
      .capacity = htobe16((uint16_t)(0)) << 8,
      .remaining = (uint8_t)(0),
    };
    crsf_.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
    lastTxStats = now;

    if (canPrint()) {
      for (int i = 0; i < NUM_DRIVES; i++) {
        Serial.printf("o%d: e%x s%d\n", i, lastHeartbeats_[i].Axis_Error, lastHeartbeats_[i].Axis_State);
      }
      Serial.println();
    }
  }
#endif

  #ifdef IS_M5
  M5.update(); //updates buttons, etc
  if (M5.BtnA.wasPressed()) {
    selectedTune_ = (selectedTune_ + 1) % (NUM_ADJUSTABLES + 1); //+1 for disabled
    redrawLCD_ = true;
  }
  #endif
  float* tunable = selectedTune_ < NUM_ADJUSTABLES? adjustables_[selectedTune_] : NULL;

  if ((now - lastDrive) > IMU_UPDATE_PERIOD) {
    updateIMU();

    float q[4] = {0};
    imuFilt_.getQuaternion(q);

    float R[3][3] = {0};
    R[0][0] = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    R[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
    R[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
    R[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
    R[1][1] = 1 - 2 * (q[1] * q[1] + q[3] * q[3]);
    R[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
    R[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
    R[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
    R[2][2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);

    float pitchFwd = (atan2(-R[2][0], R[2][2]) + PI / 2) * 180.0 / PI;

    bool isUpOnEnd = abs(pitchFwd) < (isBalancing_? MAX_TILT * 1.6 : MAX_TILT); //more tilt allowed when balancing

    //control inputs
    float fwd = 0, side = 0, yaw = 0, thr = 0;
    bool arm = lastState_;
    bool balanceChange = false;
    // --- CRSF Rx --- //

    bool csrfArm = crsf_.isLinkUp()? crsf_.getChannel(5) > 1500 : false;
    bool motionArm = lastMotionCmd_.state > 0;
    if (crsf_.isLinkUp()) { //stil use crsf for speed control
      maxSpeed_ = mapfloat(crsf_.getChannel(7), 1000, 2000, 6, 30); //aux 2: speed selection
    }

    if (crsf_.isLinkUp() && csrfArm) {
      arm = csrfArm;
      yawCtrlEnabled_ = crsf_.getChannel(6) > 1400 && crsf_.getChannel(6) < 1600;
      bool balanceModeEnabled_ = crsf_.getChannel(6) > 1600;
      bool enableAdjustment = (yawCtrlEnabled_ || balanceModeEnabled_) && (crsf_.getChannel(8) > 1500);
      maxSpeed_ = mapfloat(crsf_.getChannel(7), 1000, 2000, 6, 30); //aux 2: speed selection
      isUpOnEnd &= balanceModeEnabled_;

      fwd = mapfloat(crsf_.getChannel(2), 1000, 2000, -maxSpeed_, maxSpeed_);
      side = mapfloat(crsf_.getChannel(1), 1000, 2000, -maxSpeed_, maxSpeed_);
      yaw = mapfloat(crsf_.getChannel(4), 1000, 2000, -maxSpeed_, maxSpeed_);
      thr = mapfloat(crsf_.getChannel(3), 1000, 2000, 0.0, 1.0);

      if (enableAdjustment && tunable) {
        *tunable = pow(10, mapfloat(thr, 0, 1, -2, 2));
        if (thr < 0.01) *tunable = 0; //allow 0 values
      }

    // --- ESP-NOW Tx / Rx --- //
    } else if (lastMotionCmd_.timestamp > (now - 400) && motionArm) {
      arm = motionArm;
      if (!crsf_.isLinkUp())
        maxSpeed_ = 18;
      fwd = mapfloat(lastMotionCmd_.fwd, -1, 1, -maxSpeed_, maxSpeed_);
      yaw = mapfloat(lastMotionCmd_.yaw, -1, 1, -maxSpeed_, maxSpeed_);
      side = mapfloat(lastMotionCmd_.roll, -30, 30, -maxSpeed_, maxSpeed_); //this is in degrees tilt
      side = constrain(side, -maxSpeed_, maxSpeed_);
    } else { // no control input
      arm = false;
      yawCtrlEnabled_ = false;
      isUpOnEnd = false;
    }
    if (isBalancing_ != isUpOnEnd) {
      isBalancing_ = isUpOnEnd;
      if (canPrint()) Serial.printf("Balancing mode %s\n", isUpOnEnd? "enabled" : "disabled");
      resetPids();
      balanceChange = true;
    }

    //main control loop
    if (getValidDriveCount()) { //at least one

      //arm/disarm
      if (arm != lastState_) {
        if (canPrint())
          Serial.printf("SETTING STATE %s\n", arm? "ARM" : "DISARM");
        for (int i = 0; i < NUM_DRIVES; i++) {
          drives_[i]->setMode(arm? MotorMode::Speed : MotorMode::Disabled);
        }
        resetPids();
        lastState_ = arm? 1 : 0;
        redrawLCD_ = true;
      }

      yawCtrl_.limit = maxSpeed_; //TODO filter this
      float y = yawCtrlEnabled_? yawCtrl_.update(now, (-yaw * 100) - gyroZ) : -yaw; //convert yaw to angular rate

#if NUM_DRIVES == 3
      #define BACK 0
      #define RGHT 1
      #define LEFT 2

      fwdSpeed_ = yawSpeed_ = 0;
      for (int i = 0; i < NUM_DRIVES; i++) {
        float v = drives_[i]->getMotorState().velocity;
        if (i != BACK) {
          fwdSpeed_ += v * (i == RGHT? 1 : -1); //left is positive, right is negative
          yawSpeed_ += v;
        }
      }
      fwdSpeed_ /= 2;
      yawSpeed_ /= 2;

      if (balanceChange) {
        for (int d = 0; d < NUM_DRIVES; d++)
          if (d != BACK) //back stays in speed mode
            drives_[d]->setMode(isBalancing_? MotorMode::Current : MotorMode::Speed);
      }

      if (isBalancing_) {
        float pitchGoal = balanceSpeedCtrl_.update(now, fwdSpeed_ - fwd); //input is speed, setpoint is [user fwd]
        float torqueCmd = balanceCtrl_.update(now, pitchGoal - pitchFwd); //input is angle
        if (canPrint()) {
          Serial.printf("(fwd %06.2f)-> [-pgoal %06.2f -p %06.2f] -> torque %06.2f\n",
            fwd, pitchGoal, pitchFwd, torqueCmd);
        }
        fwd = torqueCmd; // Use torque output directly

        y = balanceYawCtrl_.update(now, (y + -side) - yawSpeed_); //input is speed, output is torque
        side = 0; //disable side
      } else {
        balanceSpeedCtrl_.reset();
        balanceCtrl_.reset();
      }
      if (arm) {
        drives_[BACK]->setSetpoint(MotorMode::Speed, isBalancing_? yawSpeed_ : (y  +   0   + side));
        drives_[LEFT]->setSetpoint(isBalancing_? MotorMode::Current : MotorMode::Speed, y  - fwd   - side * 1.33/2);
        drives_[RGHT]->setSetpoint(isBalancing_? MotorMode::Current : MotorMode::Speed, y  + fwd   - side * 1.33/2);
      }
#elif NUM_DRIVES == 4
      #define BK_L 0
      #define BK_R 1
      #define FR_R 2
      #define FR_L 3
      // yaw, fwd
      vels[BK_L] += y  +  fwd;
      vels[BK_R] += y  -  fwd;
      vels[FR_R] += y  -  fwd;
      vels[FR_L] += y  +  fwd;

#else
#warning "unsupported motor count, is: NUM_DRIVES"
#endif
      //now output the drive commands

      if (lastLinkUp_ != isLinkUp(now))
        redrawLCD_ = true;
      lastLinkUp_ = isLinkUp(now);
    } // validCount
    lastDrive = now;
  }

  if ((now - lastDraw) > 60 || redrawLCD_) {
    drawLCD(now);
    lastDraw = now;
  }

  crsf_.update();

  while (twaiInterface_.available()) {
    CanMessage message = twaiInterface_.readOne();
    bool handled = false;
    for (int i = 0; i < NUM_DRIVES; i++) {
      if (drives_[i]->handleIncoming(message.id, message.data, message.len, now))
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
  imuFilt_.updateIMU<0,'D'>(-data.gyro.y * gyroScale_, -data.gyro.x * gyroScale_, -data.gyro.z, data.accel.y, data.accel.x, data.accel.z); //acc x/y are swapped
#endif
  return true;
}

uint16_t rainbowColor(float v) {
    float h = v * 6.0;
    float x = 1.0 - fabs(fmod(h, 2.0) - 1.0);
    return lgfx::color565(
        (h < 1 ? 1 : h < 2 ? x : h < 4 ? 0 : h < 5 ? x : 1) * 255,
        (h < 1 ? x : h < 3 ? 1 : h < 4 ? x : 0) * 255,
        (h < 2 ? 0 : h < 3 ? x : h < 5 ? 1 : x) * 255
    );
}
const uint16_t SUPERDARKRED = lgfx::color565(50, 0, 0);
const uint16_t SUPERDARKBLUE = lgfx::color565(0, 0, 50);

void Controller::drawLCD(const uint32_t now) {
  lcd_->startWrite();
  auto bgRainbow = rainbowColor((now >> 2) % 1000 / 1000.0);
  auto fg = BLACK;
  auto pageBG = BLACK;
  auto validCount = getValidDriveCount();
  auto link = isLinkUp(now);
  String title = (link? "ready" : "NO LINK");
  if (!link) bgRainbow = RED;
  if (isBalancing_) title = "balancing!";
  if (!validCount) {
    title = "no drives!";
    bgRainbow = RED;
  }
  if (bgRainbow == RED) { fg = WHITE; pageBG = SUPERDARKRED; }
  if (lastState_) { title = "running"; pageBG = SUPERDARKBLUE; }

  //draw 2px line down left, right, and bottom of screen
  lcd_->fillRect(0, 0, 2, lcd_->height(), bgRainbow); //vertical left
  lcd_->fillRect(lcd_->width() - 2, 0, 2, lcd_->height(), bgRainbow); //vertical right
  lcd_->fillRect(0, lcd_->height() - 2, lcd_->width(), 2, bgRainbow); //horizontal bottom

  //show title
  lcd_->setCursor(2, 0);
  lcd_->setTextColor(fg, bgRainbow);
  lcd_->setFont(&FreeSansBold12pt7b);
  drawCentered(title.c_str(), lcd_, bgRainbow);
  lcd_->setCursor(2, lcd_->getCursorY()); //indent-in 2px

  if (redrawLCD_ || (now - lastClear) > 5000) {
    auto x = lcd_->getCursorX(), y = lcd_->getCursorY();
    lcd_->fillRect(x, y, lcd_->width() - x - 2, lcd_->height() - y - 2, pageBG);
    lastClear = now;
    redrawLCD_ = false;
  }

  // if this has a battery, show that
  auto power = M5.Power.getType();
  if (power != M5.Power.pmic_unknown) {
    lcd_->setTextColor(WHITE, BLACK);
    lcd_->setFont(&FreeMono12pt7b);
    lcd_->printf(" %0.1fV\n", M5.Power.getBatteryVoltage() / 1000.0);
  }

  //next show odrive status
#if 0
  if (validCount) {
    //show drive voltage
    //TODO! can read 0X3007 vBus(mv) register for this if the driver will let us!
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setFont(&FreeSansBold18pt7b);
    M5.Lcd.printf("%0.1fV", lastBusStatus_.Bus_Voltage);
    M5.Lcd.setFont(&FreeSansBold9pt7b); //small
    for (int i = 0; i < NUM_DRIVES; i++) {
      M5.Lcd.setTextColor(WHITE, lastHeartbeatTimes_[i] > (now - 100)? DARKGREEN : RED);
      M5.Lcd.printf("%d", i);
    }
    M5.Lcd.setFont(&FreeSansBold18pt7b);
    M5.Lcd.printf("\n"); //newline with big font
  }
#endif

  if (selectedTune_ < NUM_ADJUSTABLES) {
    M5.Lcd.setTextColor(SUPERDARKBLUE, WHITE);
    M5.Lcd.setFont(&FreeSansBold9pt7b);
    auto tuneLabel = adjNames_[selectedTune_];
    String draw = tuneLabel + ":" + String(*adjustables_[selectedTune_], 2);
    drawCentered(draw.c_str(), lcd_, WHITE);
    M5.Lcd.setCursor(2, M5.Lcd.getCursorY()); //indent-in 2px
  }

  M5.Lcd.setFont(&FreeSansBold9pt7b);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf(" spd %d", (uint8_t)maxSpeed_);
  if (yawCtrlEnabled_ || isBalancing_) {
    M5.Lcd.setTextColor(fg, bgRainbow);
    M5.Lcd.printf(" %s", yawCtrlEnabled_? "[yaw]" : "[bal]");
  }

  M5.Lcd.setFont(&FreeMono9pt7b);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("p%0.1f", imuFilt_.getPitchDegree());

  M5.Lcd.setFont(&FreeMono9pt7b);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.printf("\nva%0.1f", fwdSpeed_);

  //bottom aligned
  M5.Lcd.setFont(&Font0);
  M5.Lcd.setCursor(1, M5.Lcd.height() - 9);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf(" %.2f ", 1 / 1000.0 * now);

  for (int i = 0; i < NUM_DRIVES; i++) {
    if (!drives_[i]->getLastFaults()) continue;
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.printf("[o%d:e%x]", i, drives_[i]->getLastFaults());
  }

  M5.Lcd.endWrite();
}

void drawCentered(const char* text, lgfx::v1::LGFX_Device* lcd, uint16_t bg, uint16_t lr_padding) {
  if (!lcd) return;
  auto titlewidth = lcd->textWidth(text);
  auto titleheight = lcd->fontHeight();
  auto starty = lcd->getCursorY();
  if (titlewidth > lcd->width() - 4) {
    lcd->setFont(&FreeMono9pt7b); //smaller
    titlewidth = lcd->textWidth(text);
  }
  lcd->setCursor((lcd->width() - titlewidth) / 2, starty); //center title
  lcd->fillRect(lr_padding, starty, lcd->getCursorX(), lcd->fontHeight(), bg);
  lcd->print(text);
  lcd->fillRect(lcd->getCursorX(), starty, lcd->width() - lcd->getCursorX() - lr_padding, lcd->fontHeight(), bg);
  lcd->println();
}

