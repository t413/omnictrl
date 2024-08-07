#pragma once

#include <WString.h>
#include <can_common.h>
#include <ODriveCAN.h>
#include <AlfredoCRSF.h>

#define NUM_ODRIVES 3

class Controller {
  ODriveCanIntfWrapper commsWrapper_ = {0};
  ODriveCAN* odrives_[NUM_ODRIVES] = {0};
  uint32_t lastHeartbeatTimes_[NUM_ODRIVES] = {0};
  uint32_t lastFeedbackTimes_[NUM_ODRIVES] = {0};
  Get_Encoder_Estimates_msg_t lastFeedbacks_[NUM_ODRIVES];
  Heartbeat_msg_t lastHeartbeats_[NUM_ODRIVES];
  Get_Bus_Voltage_Current_msg_t lastBusStatus_;

  AlfredoCRSF crsf_;
  int8_t lastState_ = -1;

public:
  Controller(String version);
  ~Controller();

  void setup();
  void loop();

  void onCanMessageReceived(CAN_FRAME* frame);

  const String version_;
};
