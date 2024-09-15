#include <Arduino.h>
#include <OneButton.h>
#include <driver/twai.h>
#include <xiaomi_cybergear_driver.h>

#define RX_PIN GPIO_NUM_1
#define TX_PIN GPIO_NUM_2

#define BTN_PIN GPIO_NUM_41
#define POLLING_RATE_MS 1000
OneButton btn(BTN_PIN, true);

uint8_t CYBERGEAR_CAN_ID = 0x7F;
// uint8_t CYBERGEAR_CAN_ID = 0x80;
// uint8_t CYBERGEAR_CAN_ID = 0x02;
uint8_t MASTER_CAN_ID = 0x00;
XiaomiCyberGearDriver cybergear = XiaomiCyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);

bool isEnabled = false;
bool direction = true;
float speedDesired = 0.0;

void setup() {
  // initialize TWAI (CAN) interface to communicate with Xiaomi CyberGear
  // this needs to be called only once for any cybergear
  cybergear.init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);

  // Serial.begin(115200) will be called in cybergear.init_twai function

  cybergear.stop_motor(); //just in case
  cybergear.set_position_ref(0.0); /* set initial rotor position */

  btn.setClickMs(300);
  btn.attachClick([]() {
    if (!isEnabled) {
      if (Serial) Serial.println("Enabling motor");
      cybergear.init_motor(MODE_SPEED);
      cybergear.set_limit_speed(10.0f);
      cybergear.set_limit_current(5.0);
      cybergear.enable_motor();
      isEnabled = true;
    } else {
      if (Serial) Serial.println("Disabling motor");
      speedDesired = 0.0;
      cybergear.set_speed_ref(0.0);
      cybergear.stop_motor();
      isEnabled = false;
    }
  });
  btn.attachDoubleClick([]() {
    direction = !direction;
    speedDesired = -speedDesired;
    if (Serial) Serial.printf("Direction: %s\n", direction ? "forward" : "backward");
  });
  btn.attachLongPressStop([]() {
    //set new CAD ID
    CYBERGEAR_CAN_ID = (CYBERGEAR_CAN_ID - 2); //decrement
    Serial.println();
    Serial.println();
    Serial.printf("setting new CYBERGEAR_CAN_ID: %x\n", CYBERGEAR_CAN_ID);
    cybergear.set_motor_can_id(CYBERGEAR_CAN_ID);
    Serial.println("done");
  });
}

static void handle_rx_message(twai_message_t& message) {
  auto canid = (message.identifier & 0xFF00) >> 8;
  Serial.printf("Received message: canid: 0x%x, id: 0x%x len %d: {", canid, message.identifier, message.data_length_code);
  for (int i = 0; i < message.data_length_code; i++)
    Serial.printf("0x%x ", message.data[i]);
  Serial.println("}");
  if (canid == CYBERGEAR_CAN_ID){
    cybergear.process_message(message);
  }
}

static void check_alerts(){
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twai_status;
  twai_get_status_info(&twai_status);

  // Handle alerts
  if (Serial) {
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      Serial.println("Alert: TWAI controller has become error passive.");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      Serial.printf("Bus error count: %d\n", twai_status.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
      Serial.println("Alert: The Transmission failed.");
      Serial.printf("TX buffered: %d\t", twai_status.msgs_to_tx);
      Serial.printf("TX error: %d\t", twai_status.tx_error_counter);
      Serial.printf("TX failed: %d\n", twai_status.tx_failed_count);
    }
    if (alerts_triggered & TWAI_ALERT_TX_SUCCESS)
      Serial.printf(" > tx ack status 0x%x\n", alerts_triggered);
      //TODO maybe store that the system is working?
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }
}


uint32_t lastControl = 0;
uint32_t lastStatusRequest = 0;


void setSpeed(float speed) {
  cybergear.set_speed_ref(speed);
}

void loop() {
  const auto now = millis();
  btn.tick();

  if (now - lastControl > 50) {
    if (isEnabled) {
      setSpeed(10.0 * sin(now / 500.0));
    }
    lastControl = now;
  }

  if ((now - lastStatusRequest) > 333) {
    if (btn.isLongPressed()) {
      auto down = btn.getPressedMs();
      if (Serial) Serial.printf("Long pressed for %d ms, en%d\n", down, isEnabled);
    }

    check_alerts();
    cybergear.request_status();

    XiaomiCyberGearStatus cybergear_status = cybergear.get_status();
    if (Serial) Serial.printf("POS:%f V:%f T:%f temp:%d\n", cybergear_status.position, cybergear_status.speed, cybergear_status.torque, cybergear_status.temperature);

    lastStatusRequest = now;
  }
}
