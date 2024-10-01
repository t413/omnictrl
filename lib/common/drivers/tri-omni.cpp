#include "tri-omni.h"
#include "../controller.h"
#include "../utils.h"

#define BACK 0
#define RGHT 1
#define LEFT 2

static constexpr uint8_t ID_MAP_A[3] = { 0x7D, 0x7E, 0x7F }; //cybergear IDs, in order
static constexpr uint8_t ID_MAP_B[3] = { 0x16, 0x17, 0x18 }; //odrive robot TODO get order right

TriOmni::TriOmni() { }

bool TriOmni::canPrint() const { return ctrl_ && ctrl_->canPrint(); }

bool getDriveIDs(Controller* ctrl, uint8_t* ids) {
  auto drives = ctrl->getDrives();
  uint8_t index = 0;
  for (int d = 0; d < MAX_DRIVES; d++) {
    if (!drives[d]) continue;
    ids[index++] = drives[d]->getID();
  }
  return index == 3;
}

bool compareArraysUnordered(const uint8_t* a, const uint8_t* b, uint8_t len) {
  for (int i = 0; i < len; i++) {
    bool found = false;
    for (int j = 0; j < len; j++) {
      if (a[i] == b[j]) {
        found = true;
        break;
      }
    }
    if (!found) return false;
  }
  return true;
}

DynamicsDriver* TriOmni::canInit(Controller* ctrl) {
  ctrl_ = ctrl;
  if (ctrl->getValidDriveCount() != 3) return nullptr;
  //check IDs against ID_MAP_A and ID_MAP_B
  uint8_t ids[3] = {0};
  if (!getDriveIDs(ctrl, ids)) return nullptr;
  if (compareArraysUnordered(ids, ID_MAP_A, 3)) {
    memcpy(drivesIDs_, ids, 3);
    Serial.printf("TriOmni start A: IDs: %d, %d, %d\n", ids[0], ids[1], ids[2]);
    return this;
  } else if (compareArraysUnordered(ids, ID_MAP_B, 3)) {
    memcpy(drivesIDs_, ids, 3);
    Serial.printf("TriOmni start B: IDs: %d, %d, %d\n", ids[0], ids[1], ids[2]);
    return this;
  }
  return nullptr;
}

Adjustables TriOmni::getAdjustables() {
  return {
    { "yawP", &yawCtrl_.P },
    { "yawI", &yawCtrl_.I },
    { "yawD", &yawCtrl_.D },
    { "balP", &balanceCtrl_.P },
    { "balI", &balanceCtrl_.I },
    { "balD", &balanceCtrl_.D },
    { "balSpP", &balanceSpeedCtrl_.P },
    { "balSpI", &balanceSpeedCtrl_.I },
    { "balSpD", &balanceSpeedCtrl_.D },
    { "fwdSpeedAlpha", &filteredFwdSpeedAlpha_ },
  };
}

constexpr float MAX_TILT = 20.0;

void TriOmni::loop() {
  if (!ctrl_) return;
  auto& rx = ctrl_->getLink();
  const float** R = ctrl_->getRot();

  float pitchFwd = (atan2(-R[2][0], R[2][2]) + PI / 2) * 180.0 / PI;

  bool isUpOnEnd = abs(pitchFwd) < (isBalancing_? MAX_TILT * 1.6 : MAX_TILT); //more tilt allowed when balancing

  //control inputs
  yawCtrlEnabled_ = rx.getChannel(6) > 1400 && rx.getChannel(6) < 1600;
  bool balanceModeEnabled_ = rx.getChannel(6) > 1600;
  bool enableAdjustment = (yawCtrlEnabled_ || balanceModeEnabled_) && (rx.getChannel(8) > 1500);
  maxSpeed_ = mapfloat(rx.getChannel(7), 1000, 2000, 6, 30); //aux 2: speed selection
  isBalancing_ = (balanceModeEnabled_ && isUpOnEnd);

  float fwd = mapfloat(rx.getChannel(2), 1000, 2000, -maxSpeed_, maxSpeed_);
  float side = mapfloat(rx.getChannel(1), 1000, 2000, -maxSpeed_, maxSpeed_);
  float yaw = mapfloat(rx.getChannel(4), 1000, 2000, -maxSpeed_, maxSpeed_);
  float thr = mapfloat(rx.getChannel(3), 1000, 2000, 0.0, 1.0);

  //arm/disarm
  bool arm = rx.getChannel(5) > 1500;
  if (arm != lastArmed_) {
    if (canPrint())
      Serial.printf("SETTING STATE %s\n", arm? "ARM" : "DISARM");
    for (auto id : drivesIDs_) {
      auto drive = ctrl_->getDrive(id);
      if (!drive) continue;
      if (drive->getLastStatusTime() > 1000) continue; //skip invalid drives
      drive->setModeSpeed();
      drive->enable(arm);
      yawCtrl_.reset();
      balanceCtrl_.reset();
    }
    lastArmed_ = arm;
  }

  float vels[MAX_DRIVES] = {0};
  yawCtrl_.limit = maxSpeed_; //TODO filter this
  float y = yawCtrlEnabled_? yawCtrl_.update((-yaw * 100) - gyroZ) : -yaw; //convert yaw to angular rate

    #define BACK 0
    #define RGHT 1
    #define LEFT 2
    if (isBalancing_) {
      //balance mode
      float pitch = pitchFwd;
      float pitchGoal = balanceSpeedCtrl_.update(filteredFwdSpeed_ - fwd);
      float pctrl = balanceCtrl_.update(pitchGoal - pitch); //goal is pitch at 0
      filteredFwdSpeed_ = filteredFwdSpeed_ * (1 - filteredFwdSpeedAlpha_) + pctrl * filteredFwdSpeedAlpha_;
      if (canPrint()) {
        Serial.printf("(speed %06.2f, fwd %06.2f)-> [-pgoal %06.2f -p %06.2f] -> pctrl %06.2f\n",
          filteredFwdSpeed_, fwd, pitchGoal, pitch, pctrl);
      }
      // if (endIdx == 0) { //fwd balancing
      fwd = pctrl;
      side = 0; //disable side
    } else balanceSpeedCtrl_.reset();
    // yaw, fwd, side
    vels[BACK] = y  +   0   + side;
    vels[LEFT] = y  - fwd   - side * 1.33/2;
    vels[RGHT] = y  + fwd   - side * 1.33/2;


}
