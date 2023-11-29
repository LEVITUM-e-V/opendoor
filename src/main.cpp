#include <Arduino.h>
#include "door_actuator.h"

DoorActuator door(7, 70);

#pragma GCC diagnostic ignored "-Wunused-function"
static void IRAM_ATTR stall_guard() {
  door.notify_stalled();
}

void setup() {
  Serial.begin(115200);
  pinMode(STALL_PIN, INPUT);
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
