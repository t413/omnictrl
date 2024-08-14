#include "controller.h"
#include "utils.h"
#include <Arduino.h>
#include <M5Unified.h>
#include <WiFiUdp.h>
#include <esp32_can.h>

#define CAN_BAUDRATE 250000
#define ODRV0_NODE_ID 16

Controller::~Controller() { }

Controller::Controller(String version) :
        version_(version) {
    lastBusStatus_.Bus_Voltage = 0;
    lastBusStatus_.Bus_Current = 0;
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

  Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, 5, 6);
  crsf_.begin(Serial1);

  Wire1.begin(38, 39);
  imu_ = I2C_MPU6886(I2C_MPU6886_DEFAULT_ADDRESS, Wire1);
  imu_.begin();
  imuFilt_.begin(20);  // 20hz

  commsWrapper_.send_msg_ = [](void* intf, uint32_t id, uint8_t length, const uint8_t* data) {
    CAN_FRAME frame = CAN_FRAME();
    frame.id = id;
    frame.length = length;
    frame.extended = false;
    frame.rtr = data == NULL; //remote transmission request
    memcpy(frame.data.uint8, data, length);
    return CAN0.sendFrame(frame);
  };
  commsWrapper_.pump_events_ = [](void* intf) {
    delay(1); //can messages are async, so just wait a bit
  };

  CAN0.setCANPins(GPIO_NUM_2, GPIO_NUM_1);
  auto ret = CAN0.init(CAN_BAUDRATE);
  if (Serial && Serial.availableForWrite())
    Serial.printf("CAN0 init: %d\n", ret);
  CAN0.watchFor(); //watch for all messages
  CAN0.setCallback(0, [](CAN_FRAME* frame) {
    controller_->onCanMessageReceived(frame);
  });
  CAN0.setGeneralCallback([](CAN_FRAME* frame) {
    controller_->onCanMessageReceived(frame);
  });

  for (int i = 0; i < NUM_ODRIVES; i++) {
    odrives_[i] = new ODriveCAN(commsWrapper_, ODRV0_NODE_ID + i);

    // Register callbacks for the heartbeat and encoder feedback messages
    odrives_[i]->onFeedback([](Get_Encoder_Estimates_msg_t& msg, void* user_data) {
      uint32_t i = (uint32_t) user_data;
      controller_->lastFeedbacks_[i] = msg;
      controller_->lastFeedbackTimes_[i] = millis();
    }, (void*) i); //save index as user data
    odrives_[i]->onStatus([](Heartbeat_msg_t& msg, void* user_data) {
      uint32_t i = (uint32_t) user_data;
      controller_->lastHeartbeats_[i] = msg;
      controller_->lastHeartbeatTimes_[i] = millis();
      // Serial.printf("o%d status: %d\n", i, msg.Axis_State);
    }, (void*) i); //save index as user data
    if (Serial && Serial.availableForWrite())
      Serial.printf("- created odrive %d\n", ODRV0_NODE_ID + i);
  }

  if (Serial && Serial.availableForWrite())
    Serial.printf("ODriveCAN setup\n");

  // auto cfg = M5.config();
  M5.begin();
  M5.Power.begin();

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextColor(WHITE, BLUE);
  M5.Lcd.printf("  board %d | %d  ", M5.getBoard(), M5.Power.getType());

  if (Serial && Serial.availableForWrite())
    Serial.println("M5 setup");
  delay(100);
}

uint32_t lastDraw = 0;
uint32_t lastOdrive = 1000;
uint32_t lastPollStats = 0;
uint32_t lastTxStats = 0;
uint32_t lastClear = 0;
uint32_t lastIMU = 0;

void Controller::loop() {
  uint32_t now = millis();
  bool shouldClear = false;

  if ((now - lastIMU) > 20) {
    updateIMU();
    lastIMU = now;
  }

  if ((now - lastPollStats) > 500) {
    //read power stats from odrive
    for (int i = 0; i < NUM_ODRIVES; i++) {
      //if heartbeat is too old, skip
      if (lastHeartbeatTimes_[i] < (now - 100))
        continue;
      Get_Bus_Voltage_Current_msg_t pkt;
      if (odrives_[i]->getBusVI(pkt, 20)) {
        lastBusStatus_ = pkt;
        if (Serial && Serial.availableForWrite())
          Serial.printf("bus%d:%0.3f;cur%d:%0.3f\n", i, pkt.Bus_Voltage, i, pkt.Bus_Current);
        break; //only read one
      }
    }
    lastPollStats = now;
  }

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
      for (int i = 0; i < NUM_ODRIVES; i++) {
        Serial.printf("o%d: e%x s%d\n", i, lastHeartbeats_[i].Axis_Error, lastHeartbeats_[i].Axis_State);
      }
      Serial.println();
    }
  }

  #define NUM_TUNABLES 3
  float* tuneables[NUM_TUNABLES] = { &yawCtrl_.P, &yawCtrl_.I, &yawCtrl_.D };
  char tuneableLabels[NUM_TUNABLES] = { 'P', 'I', 'D' };

  if (M5.BtnA.wasPressed()) {
    selectedTune_ = (selectedTune_ + 1) % (NUM_TUNABLES + 1); //+1 for disabled
    shouldClear = true;
  }
  float* tunable = selectedTune_ < NUM_TUNABLES? tuneables[selectedTune_] : NULL;
  char tuneLabel = tuneableLabels[selectedTune_ % NUM_TUNABLES];

  if ((now - lastOdrive) > 20) {

    uint8_t validCount = 0;
    //update each drive status
    for (int i = 0; i < NUM_ODRIVES; i++) {
      bool haveHeartbeat = lastHeartbeatTimes_[i] > (now - 100);
      if (haveHeartbeat) {
        Get_Encoder_Estimates_msg_t pkt;
        odrives_[i]->getFeedback(pkt, 0); //async request for feedback
        validCount++;
      }
    }
    //TODO check for issues

    //control inputs
    maxSpeed_ = mapfloat(crsf_.getChannel(7), 1000, 2000, 2, 6); //aux 2: speed selection

    float fwd = mapfloat(crsf_.getChannel(2), 1000, 2000, -maxSpeed_, maxSpeed_);
    float side = mapfloat(crsf_.getChannel(1), 1000, 2000, -maxSpeed_, maxSpeed_);
    float yaw = mapfloat(crsf_.getChannel(4), 1000, 2000, -maxSpeed_, maxSpeed_);
    float thr = mapfloat(crsf_.getChannel(3), 1000, 2000, 0.0, 1.0);
    bool enableAdjustment = crsf_.getChannel(8) > 1500;
    yawCtrlEnabled_ = crsf_.getChannel(6) < 1600 || enableAdjustment;

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
        for (int i = 0; i < NUM_ODRIVES; i++) {
          odrives_[i]->clearErrors();
          odrives_[i]->setState(arm? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE);
          yawCtrl_.reset();
        }
        lastState_ = arm? 1 : 0;
        shouldClear = true;
      }

      //mix into three omni-wheeld drive outputs
      float vels[3] = {0, 0, 0}; //back, left, right
      #define BACK 0
      #define LEFT 1
      #define RGHT 2

      // yaw
      yawCtrl_.limit = maxSpeed_;
      float y = yawCtrlEnabled_? yawCtrl_.update((-yaw * 100) - gyroZ) : -yaw; //convert yaw to angular rate
      vels[BACK] += y;
      vels[LEFT] += y;
      vels[RGHT] += y;

      // fwd
      vels[LEFT] += -fwd;
      vels[RGHT] += fwd;

      // side
      vels[BACK] +=   side * 2;
      vels[LEFT] += - side * 1.33;
      vels[RGHT] += - side * 1.33;

      //now output the drive commands

      if (arm) {
        for (int i = 0; i < NUM_ODRIVES; i++) {
          odrives_[i]->setVelocity(vels[i], 0);
        }
      }

      if (Serial && Serial.availableForWrite()) {
        Serial.printf("r:%0.2f;p:%0.2f;y:%0.2f\n", imuFilt_.getRoll(), imuFilt_.getPitch(), imuFilt_.getYaw());
        Serial.printf("v0:%0.2f;v1:%0.2f;v2:%0.2f\n", vels[0], vels[1], vels[2]);
        for (int i = 0; i < NUM_ODRIVES; i++)
          Serial.printf("pos%d:%0.2f;vel%d:%0.2f\n", i, lastFeedbacks_[i].Pos_Estimate, i, lastFeedbacks_[i].Vel_Estimate);
      }

    }

    lastOdrive = now;
  }

  if ((now - lastDraw) > 60 || shouldClear) {
    M5.update(); //updates buttons
    M5.Lcd.startWrite();
    if (shouldClear || (now - lastClear) > 5000) {
      M5.Lcd.fillScreen(lastState_ > 0? DARKGREEN : BLACK);
      lastClear = now;
    }

    //first show crsf connection status
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextColor(WHITE, crsf_.isLinkUp()? BLUE : RED);
    M5.Lcd.setFont(&FreeMono12pt7b);
    M5.Lcd.printf(" %s \n", crsf_.isLinkUp()? "link ok" : "NO LINK");

    //next show odrive status
    uint8_t validDrives = 0;
    for (int i = 0; i < NUM_ODRIVES; i++)
      if (lastHeartbeatTimes_[i] > (now - 100))
        validDrives++;
    if (validDrives) {
      //show drive voltage
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setFont(&FreeSansBold18pt7b);
      M5.Lcd.printf("%0.1fV", lastBusStatus_.Bus_Voltage);
      M5.Lcd.setFont(&FreeSansBold9pt7b); //small
      for (int i = 0; i < NUM_ODRIVES; i++) {
        M5.Lcd.setTextColor(WHITE, lastHeartbeatTimes_[i] > (now - 100)? DARKGREEN : RED);
        M5.Lcd.printf("%d", i);
      }
      M5.Lcd.setFont(&FreeSansBold18pt7b);
      M5.Lcd.printf("\n"); //newline with big font
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
    M5.Lcd.printf(" spd %d %s", (uint8_t)maxSpeed_, yawCtrlEnabled_? "[yaw]" : "");

    //bottom aligned
    M5.Lcd.setFont(&Font0);
    M5.Lcd.setCursor(0, M5.Lcd.height() - 8);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf(" %.2f ", 1 / 1000.0 * now);

    for (int i = 0; i < NUM_ODRIVES; i++) {
      auto err = lastHeartbeats_[i].Axis_Error;
      if (!err) continue;
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.printf("[o%d:e%x]", i, err);
    }

    M5.Lcd.endWrite();
    lastDraw = now;
  }

  crsf_.update();
}

bool Controller::updateIMU() {
  float v[6] = {0};
  imu_.getGyro(&v[0], &v[1], &v[2]);
  imu_.getAccel(&v[3], &v[4], &v[5]);
  imuFilt_.updateIMU(v[0], v[1], v[2], v[3], v[4], v[5]);
  gyroZ = v[2];
  return true;
}

void Controller::onCanMessageReceived(CAN_FRAME* frame) {
  // Serial.printf(" > rx id=%d, len=%d\n", frame->id, frame->length);
  for (int i = 0; i < NUM_ODRIVES; i++)
    odrives_[i]->onReceive(frame->id, frame->length, frame->data.uint8);
}

