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

#define MAX_TILT 30.0
constexpr uint32_t IMU_UPDATE_PERIOD = 20; //ms
#define LOW_BATTERY_VOLTAGE 21.0

bool canPrint() {
  #if ARDUINO_USB_CDC_ON_BOOT
  return Serial && Serial.availableForWrite();
  #else
  return true;
  #endif
}


CanEsp32Twai twaiInterface_;

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

void Controller::handleRxPacket(const uint8_t* mac, const uint8_t* buf, uint8_t len) {
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

float blend(float a, float b, float t) {
  t = constrain(t, 0.0f, 1.0f);
  return (1.0f - t) * a + t * b;
}

uint32_t lastDraw = 0;
uint32_t lastDrive = 1000;
uint32_t lastPollStats = 0;
uint32_t lastRefreshDrives = 0;
uint32_t lastTxStats = 0;
uint32_t lastClear = 0;

void Controller::loop() {
  uint32_t now = millis();

  if ((now - lastPollStats) > (lastLinkUp_? 200 : 500)) {
    uint32_t latest = 0;
    for (int i = 0; i < NUM_DRIVES; i++) {
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
    if (telem_.vbus < (LOW_BATTERY_VOLTAGE - 0.2) && enabled_) {
      for (int i = 0; i < NUM_DRIVES; i++)
          drives_[i]->setMode(MotorMode::Disabled);
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
    if (canPrint() && !lastLinkUp_)
      Serial.println("MAC: " + WiFi.macAddress());
  }
  if (((now - lastTxStats) > 100) && crsf_.isLinkUp()) {
    crsf_sensor_battery_t crsfBatt = {
      .voltage = htobe16((uint16_t)(telem_.vbus * 10.0)),
      .current = htobe16((uint16_t)(0 * 10.0)),
      .capacity = htobe16((uint16_t)(0)) << 8,
      .remaining = (uint8_t)(0),
    };
    crsf_.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
    lastTxStats = now;
  }

  #ifdef IS_M5
  M5.update(); //updates buttons, etc
  if (M5.BtnA.wasPressed()) {
    selectedTune_ = (selectedTune_ + 1) % (NUM_ADJUSTABLES + 1); //+1 for disabled
    if (selectedTune_ < NUM_ADJUSTABLES) {
      telem_.adjusting = *adjustables_[selectedTune_];
      strncpy(telem_.adjustSrc, adjNames_[selectedTune_].c_str(), sizeof(telem_.adjustSrc) - 1);
    } else {
      telem_.adjusting = 0.0;
      telem_.adjustSrc[0] = '\0';
    }
    display_.requestRedraw();
  }
  #endif
  float* tunable = selectedTune_ < NUM_ADJUSTABLES? adjustables_[selectedTune_] : NULL;

  if ((now - lastRefreshDrives) > (IMU_UPDATE_PERIOD - 2)) {
    for (int i = 0; i < NUM_DRIVES; i++) {
      drives_[i]->requestStatus();
    }
    lastRefreshDrives = now; //also updated below, to keep in sync with IMU updates
  }

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
    bool isUpOnEnd = abs(pitchFwd) < MAX_TILT; //more tilt allowed when balancing
    bool newbalance = isBalancing_ || isUpOnEnd;

    //control inputs
    float fwd = 0, side = 0, yaw = 0, thr = 0;
    bool arm = enabled_;
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
      newbalance &= balanceModeEnabled_;

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
      newbalance = false;
    }
    if (isBalancing_ && newbalance && !isUpOnEnd) {
      if ((now - lastBalanceChange_) > 2000) //extra leeway when getting going
        newbalance = false;
      else if (abs(pitchFwd) > MAX_TILT * 1.2)
        newbalance = false; //way too much tilt
    }
    if (isBalancing_ != newbalance) {
      isBalancing_ = newbalance;
      if (canPrint()) Serial.printf("Balancing mode %s\n", isBalancing_? "enabled" : "disabled");
      resetPids();
      lastBalanceChange_ = now;
    }

    //main control loop
    if (getValidDriveCount()) { //at least one

      //arm/disarm
      if (arm != enabled_) {
        if (canPrint())
          Serial.printf("SETTING STATE %s\n", arm? "ARM" : "DISARM");
        for (int i = 0; i < NUM_DRIVES; i++) {
          drives_[i]->setMode(arm? MotorMode::Speed : MotorMode::Disabled);
        }
        resetPids();
        enabled_ = arm;
        display_.requestRedraw();
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

      if (lastBalanceChange_ == now) {
        for (int d = 0; d < NUM_DRIVES; d++)
          if (d != BACK) //back stays in speed mode
            drives_[d]->setMode(isBalancing_? MotorMode::Current : MotorMode::Speed);
      }

      if (isBalancing_) {
        // Blend between angle-control and odometry speed control
        const uint32_t balanceChangeDuration = 2000; //ms
        float pitchGoal = balanceSpeedCtrl_.update(now, fwdSpeed_ - fwd);
        if ((now - lastBalanceChange_) < balanceChangeDuration) {
          const float blendT = (now - lastBalanceChange_) / (float)balanceChangeDuration;
          pitchGoal = blend( -fwd, pitchGoal, blendT);
          if (blendT < 0.5f)
            balanceSpeedCtrl_.reset(); //prevent rampup while not in control
        }
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
        display_.requestRedraw();
      lastLinkUp_ = isLinkUp(now);
    } // validCount
    telem_.pitch = pitchFwd; //save for next loop
    lastDrive = now;
    lastRefreshDrives = now; //keep drive refresh in sync with this task
  }

  if ((now - lastDraw) > 60 || display_.isRedrawRequired()) {
    uint8_t rot = (isBalancing_ || (abs(telem_.pitch) < MAX_TILT)) ? 0 : 2;
    if (display_.getLCD()->getRotation() != rot) {
      display_.getLCD()->setRotation(rot);
      display_.requestRedraw();
    }
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

void Controller::drawLCD(const uint32_t now) {
  display_.startFrame();
  auto bgRainbow = display_.timeRainbow(now);
  auto fg = BLACK;
  auto pageBG = BLACK;
  auto validCount = getValidDriveCount();
  String title = isBalancing_? "woah." : enabled_? "wee!" : "howdy";
  String err;
  if (telem_.vbus > 0.1 && telem_.vbus < LOW_BATTERY_VOLTAGE) err = "low batt!";
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
