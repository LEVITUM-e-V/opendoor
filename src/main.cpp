#include <Arduino.h>
#include "door_actuator.h"

DoorActuator door(7, 70);
void setup() {
  Serial.begin(115200);
  pinMode(STALL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stall_guard, RISING);
  door.setup();
}

void loop() {
  while (1) {
    door.rotate_infinite(Direction::OPEN);
    delay(2000);
    door.rotate_infinite(Direction::CLOSE);
    delay(2000);
  }
}
