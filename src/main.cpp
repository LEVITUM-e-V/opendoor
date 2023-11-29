#include <Arduino.h>
#include "door_actuator.h"

DoorActuator door(&Serial2, 7, 70);

#pragma GCC diagnostic ignored "-Wunused-function"
static void IRAM_ATTR stall_guard() {
  door.notify_stalled();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); // HW UART drivers
  pinMode(STALL_PIN, INPUT);
  while(!Serial2); //wait for hardware serial
  door.setup();
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stall_guard, RISING);
}

void loop() {
  while (1) {
    door.rotate_infinite(Direction::OPEN);
    delay(2000);
    door.rotate_infinite(Direction::CLOSE);
    delay(2000);
  }
}
