/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 *
 * source: https://github.com/teemuatlut/TMCStepper/tree/master
 */

#include <Arduino.h>
#include <TMCStepper.h>

#define EN_PIN           19
#define STEP_PIN         18
#define DIR_PIN          5
#define SERIAL_PORT Serial2
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2


#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware

  Serial.begin(115200);
  while (!Serial);
  Serial.println("good morning campers");

  SERIAL_PORT.begin(19200);
  while (!SERIAL_PORT);
  driver.begin();
  driver.toff(5);
  driver.rms_current(600);
  driver.microsteps(16);
//driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
//driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop

  delay(20);
  Serial.println("Driver version:");
  Serial.println(driver.version(), HEX);
}

bool shaft = false;

void loop() {
  // Run 5000 steps and switch direction in software
  for (uint16_t i = 5000; i>0; i--) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(160);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(160);
  }
  shaft = !shaft;
  driver.shaft(shaft);
}
