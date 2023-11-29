#include <Arduino.h>
#include "door_actuator.h"

DoorActuator door(&Serial2, 8, 70);

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
  int steps;
  while (1) {
    steps = door.rotate(73000, Direction::OPEN);
    Serial.print("steps: ");
    Serial.println(steps);
    delay(2000);
    steps = door.rotate(73000, Direction::CLOSE);
    Serial.print("steps: ");
    Serial.println(steps);
    delay(2000);
  }
}
