#include "controller.h"
#include "utils.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <WiFiUdp.h>

const uint8_t DRIVE_IDS[NUM_DRIVES] = {0x7D, 0x7E, 0x7F};
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

void DriveCtx::enable(bool enable) {
  if (enable) {
    driver_.init_motor(MODE_SPEED);
    driver_.enable_motor();
  } else {
    driver_.set_speed_ref(0.0);
    driver_.stop_motor();
  }
}

void DriveCtx::setSpeed(float speed) { driver_.set_speed_ref(speed); }

void DriveCtx::requestStatus() {
  if (Serial) Serial.printf("[req%x]", id_);
  driver_.request_status();
}

XiaomiCyberGearStatus DriveCtx::getStatus() const { return driver_.get_status(); }

bool DriveCtx::handle(twai_message_t& message) {
  uint8_t msgtype = (message.identifier & 0xFF000000) >> 24; //bits 24-28
  uint8_t driveid = (message.identifier & 0x0000FF00) >> 8; //bits 8-15
  if (driveid != id_) return false;
  if (msgtype == CMD_REQUEST) {
    uint8_t mode  = (message.identifier & 0x00C00000) >> 22; //bits 22-23 = [mode]
    lastFaults_   = (message.identifier & 0x003F0000) >> 16; //bits 16-21 = [Uncalibrated, hall, magsense, overtemp, overcurrent, undervolt]
    driver_.process_message(message);
    enabled_ = (mode == 2);
    if (Serial) Serial.print(".");
    lastStatus_ = millis();
    return true;
  } else if (msgtype == 0x12 || msgtype == 0x11) { //param set / get
    //ok!
    if (Serial) {
      Serial.printf("drive %x: param msg %x: {", id_, message.identifier);
      for (int i = 0; i < message.data_length_code; i++)
        Serial.printf("0x%x, ", message.data[i]);
      Serial.println("}");
    }
  } else if (msgtype == 0x15) { //fault message decimal 21
    if (Serial) {
      Serial.printf("drive %x: fault message %x: {", id_, message.identifier);
      for (int i = 0; i < message.data_length_code; i++)
        Serial.printf("0x%x, ", message.data[i]);
      Serial.println("}");
    }
    return true;
  } else {
    if (Serial)
      Serial.printf("drive %x: unknown message type %d (header 0x%x)\n", id_, msgtype, message.identifier);
    return true; //still was for us ..
  }
  return false;
}


// -------------------- //
// ---- Controller ---- //
// -------------------- //

Controller::~Controller() { }

Controller::Controller(String version) :
        version_(version) {
}

static Controller* controller_ = nullptr;

void Controller::setup() {
  controller_ = this;
  //only use the virtual serial port if it's available
  if (Serial && Serial.availableForWrite()) {
    Serial.begin(115200);
    Serial.setTimeout(10); //very fast, need to keep the ctrl loop running
    Serial.println("Controller setup");
    Serial.flush();
  }
  delay(100);

  Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  crsf_.begin(Serial1);


  for (int i = 0; i < NUM_DRIVES; i++) {
    drives_[i].id_ = DRIVE_IDS[i];
    drives_[i].driver_ = XiaomiCyberGearDriver(drives_[i].id_, 0x00); //id, master id
  }

  drives_[0].driver_.init_twai(PIN_CAN_RX, PIN_CAN_TX);

  for (int i = 0; i < NUM_DRIVES; i++) {
    drives_[i].enable(false); //disable all drives
    drives_[i].driver_.set_position_ref(0.0); // set initial rotor position
  }

  if (Serial && Serial.availableForWrite())
    Serial.printf("finished CAN setup\n");

  // auto cfg = M5.config();
  M5.begin();
  M5.Power.begin();
  if (M5.Imu.isEnabled()) {
    M5.Imu.loadOffsetFromNVS();
    // M5.Imu.setAxisOrderRightHanded( ??
    imuFilt_.setFrequency(50); //50Hz, 20ms
  }

  M5.Lcd.setRotation(2); //flip
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextColor(WHITE, BLUE);
  M5.Lcd.printf("  board %d | %d  ", M5.getBoard(), M5.Power.getType());

  if (Serial && Serial.availableForWrite())
    Serial.println("M5 setup");
  delay(100);
}

lgfx::rgb888_t rainbowColor(float r) {
  float r1 = 0.0, g1 = 0.0, b1 = 0.0;
  if (r < 0.5) {
    r1 = 1.0 - r * 2.0;
    g1 = r * 2.0;
  } else {
    g1 = 1.0 - (r - 0.5) * 2.0;
    b1 = (r - 0.5) * 2.0;
  }
  return lgfx::rgb888_t(r1 * 255, g1 * 255, b1 * 255);
}

uint32_t lastDraw = 0;
uint32_t lastDrive = 1000;
uint32_t lastPollStats = 0;
uint32_t lastTxStats = 0;
uint32_t lastClear = 0;

void Controller::loop() {
  M5.update(); //updates buttons, etc
  uint32_t now = millis();

  if ((now - lastPollStats) > 500) {
    for (int i = 0; i < NUM_DRIVES; i++) {
      drives_[i].requestStatus();
    }
    //TODO read power stats from drive

    lastPollStats = now;
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

    if (Serial && Serial.availableForWrite()) {
      for (int i = 0; i < NUM_DRIVES; i++) {
        Serial.printf("o%d: e%x s%d\n", i, lastHeartbeats_[i].Axis_Error, lastHeartbeats_[i].Axis_State);
      }
      Serial.println();
    }
  }
#endif

  #define NUM_TUNABLES 3
  float* tuneables[NUM_TUNABLES] = { &balanceCtrl_.P, &balanceCtrl_.I, &balanceCtrl_.D };
  char tuneableLabels[NUM_TUNABLES] = { 'P', 'I', 'D' };

  if (M5.BtnA.wasPressed()) {
    selectedTune_ = (selectedTune_ + 1) % (NUM_TUNABLES + 1); //+1 for disabled
    redrawLCD_ = true;
  }
  float* tunable = selectedTune_ < NUM_TUNABLES? tuneables[selectedTune_] : NULL;
  char tuneLabel = tuneableLabels[selectedTune_ % NUM_TUNABLES];

  uint8_t validCount = 0;
  //update each drive status
  for (int i = 0; i < NUM_DRIVES; i++) {
    bool hasRecent = (now - drives_[i].lastStatus_) < 1000;
    if (hasRecent) validCount++;
  }

  if ((now - lastDrive) > 20) {
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

    bool isUpOnEnd = abs(pitchFwd) < (isBalancing_? MAX_TILT * 1.4 : MAX_TILT); //more tilt allowed when balancing

    //control inputs
    yawCtrlEnabled_ = crsf_.getChannel(6) > 1400 && crsf_.getChannel(6) < 1600;
    bool balanceModeEnabled_ = crsf_.getChannel(6) > 1600;
    bool enableAdjustment = (yawCtrlEnabled_ || balanceModeEnabled_) && (crsf_.getChannel(8) > 1500);
    maxSpeed_ = mapfloat(crsf_.getChannel(7), 1000, 2000, 6, 30); //aux 2: speed selection
    isBalancing_ = (balanceModeEnabled_ && isUpOnEnd);
    if (isBalancing_) maxSpeed_ = MAX_TILT;

    float fwd = mapfloat(crsf_.getChannel(2), 1000, 2000, -maxSpeed_, maxSpeed_);
    float side = mapfloat(crsf_.getChannel(1), 1000, 2000, -maxSpeed_, maxSpeed_);
    float yaw = mapfloat(crsf_.getChannel(4), 1000, 2000, -maxSpeed_, maxSpeed_);
    float thr = mapfloat(crsf_.getChannel(3), 1000, 2000, 0.0, 1.0);

    if (enableAdjustment && tunable) {
      // *tunable = thr / 10.0; //linear
      //thr is 0-1, so we want to map it to 0.01-100
      *tunable = pow(10, mapfloat(thr, 0, 1, -2, 2));
      //allow 0 values
      if (thr < 0.01) *tunable = 0;
    }

    //main control loop
    if (validCount) { //at least one

      //arm/disarm
      bool arm = crsf_.getChannel(5) > 1500;
      if (arm != lastState_) {
        if (Serial && Serial.availableForWrite())
          Serial.printf("SETTING STATE %s\n", arm? "ARM" : "DISARM");
        for (int i = 0; i < NUM_DRIVES; i++) {
          drives_[i].enable(arm);
          yawCtrl_.reset();
          balanceCtrl_.reset();
        }
        lastState_ = arm? 1 : 0;
        redrawLCD_ = true;
      }

      float vels[NUM_MOTORS] = {0};
      yawCtrl_.limit = maxSpeed_; //TODO filter this
      float y = yawCtrlEnabled_? yawCtrl_.update((-yaw * 100) - gyroZ) : -yaw; //convert yaw to angular rate

#if NUM_MOTORS == 3
      #define BACK 0
      #define RGHT 1
      #define LEFT 2
      if (isBalancing_) {
        //balance mode
        float pitch = pitchFwd; //endPitches[endIdx];
        float pctrl = balanceCtrl_.update(-fwd - pitch); //goal is pitch at 0
        if (Serial) {
          Serial.printf("p [%06.2f - %06.2f] ->\t%06.2f\n", fwd, pitch, pctrl);
        }
        // if (endIdx == 0) { //fwd balancing
        fwd = pctrl;
        side = 0; //disable side
      }
      // yaw, fwd, side
      vels[BACK] = y  +   0   + side;
      vels[LEFT] = y  - fwd   - side * 1.33/2;
      vels[RGHT] = y  + fwd   - side * 1.33/2;
#elif NUM_MOTORS == 4
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
#error "unsupported motor count, is: NUM_MOTORS"
#endif
      //now output the drive commands

      if (arm) {
        for (int i = 0; i < NUM_DRIVES; i++) {
          if (i < NUM_MOTORS)
            drives_[i].setSpeed(vels[i]);
        }
      }

      if (lastLinkUp_ != crsf_.isLinkUp())
        redrawLCD_ = true;
      lastLinkUp_ = crsf_.isLinkUp();
    } // validCount
    lastDrive = now;
  }

  if ((now - lastDraw) > 60 || redrawLCD_) {
    M5.update(); //updates buttons
    M5.Lcd.startWrite();
    auto bgRainbow = rainbowColor((now >> 3) % 1000 / 1000.0).get();
    if (redrawLCD_ || (now - lastClear) > 5000) {
      auto color = lastState_? DARKGREEN : BLACK;
      M5.Lcd.fillScreen(color);
      lastClear = now;
      redrawLCD_ = false;
    }
    //draw 1px line down left, right, and bottom of screen
    M5.Lcd.drawLine(0, 0, 0, M5.Lcd.height(), bgRainbow);
    M5.Lcd.drawLine(M5.Lcd.width() - 1, 0, M5.Lcd.width() - 1, M5.Lcd.height(), bgRainbow);
    M5.Lcd.drawLine(0, M5.Lcd.height() - 1, M5.Lcd.width(), M5.Lcd.height() - 1, bgRainbow);

    M5.Lcd.setCursor(1, 0);

    // if this has a battery, show that
    auto power = M5.Power.getType();
    if (power != M5.Power.pmic_unknown) {
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setFont(&FreeMono12pt7b);
      M5.Lcd.printf(" %0.1fV\n", M5.Power.getBatteryVoltage() / 1000.0);
    }

    //show crsf connection status
    M5.Lcd.setTextColor(WHITE, crsf_.isLinkUp()? bgRainbow : RED);
    M5.Lcd.setFont(&FreeMono12pt7b);
    M5.Lcd.printf(" %s \n", crsf_.isLinkUp()? "link ok" : "NO LINK");

    //next show odrive status
    if (validCount) {
#if 0
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
#endif
    } else {
      M5.Lcd.setTextColor(WHITE, RED);
      M5.Lcd.setFont(&FreeSansBold12pt7b);
      M5.Lcd.printf("no drives\n");
    }

    if (tunable) {
      M5.Lcd.setTextColor(DARKGREEN, WHITE);
      M5.Lcd.setFont(&FreeMonoBoldOblique9pt7b);
      M5.Lcd.printf("%c: %0.3f\n", tuneLabel, *tunable);
    }

    M5.Lcd.setFont(&FreeSansBold9pt7b);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf(" spd %d", (uint8_t)maxSpeed_);
    if (yawCtrlEnabled_ || isBalancing_) {
      M5.Lcd.setTextColor(WHITE, bgRainbow);
      M5.Lcd.printf(" %s", yawCtrlEnabled_? "[yaw]" : "[bal]");
    }

    //bottom aligned
    M5.Lcd.setFont(&Font0);
    M5.Lcd.setCursor(1, M5.Lcd.height() - 9);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf(" %.2f ", 1 / 1000.0 * now);

    for (int i = 0; i < NUM_DRIVES; i++) {
      if (!drives_[i].lastFaults_) continue;
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.printf("[o%d:e%x]", i, drives_[i].lastFaults_);
    }

    M5.Lcd.endWrite();
    lastDraw = now;
  }

  crsf_.update();

  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(1));

  if (Serial) {
    twai_status_info_t twai_status;
    twai_get_status_info(&twai_status);
    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
      Serial.println("CAN Error: TWAI controller has become error passive.");
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
      Serial.printf("CAN bus error count: %d\n", twai_status.bus_error_count);
    if (alerts_triggered & TWAI_ALERT_TX_FAILED)
      Serial.printf("CAN transmission failed! buffered: %d, error: %d, failed: %d", twai_status.msgs_to_tx, twai_status.tx_error_counter, twai_status.tx_failed_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      bool handled = false;
      for (int i = 0; i < NUM_DRIVES; i++) {
        if (drives_[i].handle(message))
          handled = true;
      }
      if (!handled && Serial) {
        Serial.printf("unhandled message id=%x len=%d: {", message.identifier, message.data_length_code);
        for (int i = 0; i < message.data_length_code; i++)
          Serial.printf("0x%x, ", message.data[i]);
        Serial.println("}");
      }
    }
  }
}

bool Controller::updateIMU() {
  auto res = M5.Imu.isEnabled()? M5.Imu.update() : 0;
  if (!res) return false;
  auto data = M5.Imu.getImuData(); //no mag data it seems, sadly
  imuFilt_.updateIMU<0,'D'>(-data.gyro.y, -data.gyro.x, -data.gyro.z, data.accel.y, data.accel.x, data.accel.z); //acc x/y are swapped
  return true;
}

