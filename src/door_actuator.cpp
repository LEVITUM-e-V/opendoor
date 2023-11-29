#include <Arduino.h>
#include <TMCStepper.h>
#include <cstdint>

#include "door_actuator.h"

auto SERIAL_PORT{Serial2};

bool DoorActuator::setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  digitalWrite(EN_PIN, HIGH); // disable driver for now


  _driver.begin();
  _driver.toff(4);
  _driver.blank_time(24);
  _driver.I_scale_analog(false);
  _driver.internal_Rsense(false); 
  _driver.mstep_reg_select(true);
  _driver.rms_current(400);
  _driver.microsteps(64);

  // driver.en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
  _driver.en_spreadCycle(false);
  _driver.pwm_autoscale(true);
  _driver.pdn_disable(false);
  _driver.semin(0);
  _driver.semax(2);
  _driver.sedn(0b01);
  _driver.shaft(false);
  _driver.TCOOLTHRS(0xFFFFF);
  _driver.TPWMTHRS(0);
  _driver.SGTHRS(_stall_thrs);

  Serial.print("driver version: ");
  Serial.println(_driver.version());
  return true;
}

void DoorActuator::notify_stalled() {
  _stalled = true;
}

int DoorActuator::rotate(
    std::optional<const uint32_t> steps,
    const Direction direction,
    const bool stallguard
    ) {
  uint32_t step_counter = 0;

  _driver.shaft(direction != Direction::OPEN);
  _driver.rms_current(400);
  _driver.TPWMTHRS(0);
  _driver.SGTHRS(_stall_thrs);
  digitalWrite(EN_PIN, LOW);

  _stalled = false;
  while (!_stalled || !stallguard) {
    if (steps && step_counter >= steps.value()) {
      break;
    }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(_delay_step);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(_delay_step);
    ++step_counter;
  }

  digitalWrite(EN_PIN, HIGH);
  return step_counter;
}
