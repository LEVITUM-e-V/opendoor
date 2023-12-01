#include <Arduino.h>
#include "door_actuator.h"

DoorActuator door(&Serial2, 10);

#pragma GCC diagnostic ignored "-Wunused-function"
static void IRAM_ATTR stall_guard() {
  door.notify_stalled();
}

bool homed = false;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); // HW UART drivers
  pinMode(STALL_PIN, INPUT);
  while(!Serial2); //wait for hardware serial
  door.setup();
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stall_guard, RISING);

  homed = door.home();
}

void loop() {
  if (!homed) {
    delay(2000);
    return;
  }
  while (0) {
    Serial.print("opening... ");
    door.open();
    Serial.println("done");
    delay(2000);
    Serial.print("closing... ");
    door.close();
    Serial.println("done");
    delay(2000);
  }
}
