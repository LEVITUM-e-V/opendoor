#include <Arduino.h>
#include <TMCStepper.h>

#include "door_actuator.h"

// source: https://github.com/teemuatlut/TMCStepper/issues/178

// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
constexpr auto EN_PIN{19};
constexpr auto STEP_PIN{18};
constexpr auto STALL_PIN{5};
constexpr auto DRIVER_ADDRESS{0b00};
constexpr auto R_SENSE{0.11f};
constexpr auto STALL_VALUE{12}; // [0..255], higher -> increases stall sensitivity

auto SERIAL_PORT{Serial2};
int stepTime = 160;

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
bool shaftVal = false;
bool stalled = false;

void stallInterruptX() { // flag set when motor stalls
  stalled = true;
}

void run_until_stalled(bool dir);

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  // shaft direction controlled through uart: driver.shaft(true or false)
  pinMode(STALL_PIN, INPUT);

  Serial.begin(115200);
  SERIAL_PORT.begin(115200); // HW UART drivers

  driver.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver.toff(4); // Enables driver in software, changed from 5
  driver.blank_time(24);
  driver.rms_current(600); // Set motor RMS current
  driver.microsteps(16);   // Set microsteps to 1/16th

  // driver.en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
  // driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true); // Needed for stealthChop
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.shaft(shaftVal);
  // TCOOLTHRS needs to be set for stallgaurd to work //
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.SGTHRS(STALL_VALUE);
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stallInterruptX, RISING);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware
}

void loop() {
  while (1) {
    run_until_stalled(true);
    delay(2000);
    run_until_stalled(false);
    delay(5000);
  }
}

void run_until_stalled(bool dir) {
  int delay = 500;
  Serial.println("running until stalled");
  digitalWrite(EN_PIN, LOW);
  driver.shaft(dir);
  while (!stalled) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delay);
  }
  stalled = false;
  digitalWrite(EN_PIN, HIGH);
  Serial.println("stalled");
}
