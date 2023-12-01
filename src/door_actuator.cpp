#include <Arduino.h>
#include <TMCStepper.h>
#include <cstdint>
#include <optional>

#include "door_actuator.h"

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

uint32_t DoorActuator::rotate(
    std::optional<const uint32_t> steps,
    const DoorPosition direction,
    const bool stallguard,
    const uint32_t step_delay
    ) {
  uint32_t step_counter = 0;

  _driver.shaft(direction != DoorPosition::OPEN);
  _driver.rms_current(400);
  _driver.TPWMTHRS(0);
  _driver.SGTHRS(_stall_thrs);
  digitalWrite(EN_PIN, LOW);

  _stalled = false;
  while (!(stallguard && _stalled)) {
    if (steps && step_counter >= steps.value()) {
      break;
    }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(step_delay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(step_delay);
    ++step_counter;
  }

  digitalWrite(EN_PIN, HIGH);
  return step_counter;
}

bool DoorActuator::home(bool force) {
  _position = std::nullopt;
  Serial.println("trying to go to close position");
  this->rotate(std::nullopt, DoorPosition::CLOSE, true);
  Serial.println("counting steps to open position");
  const uint32_t steps = this->rotate(std::nullopt, DoorPosition::OPEN, true);

  Serial.print("steps: ");
  Serial.println(steps);
  if (steps < _way_steps) {
    Serial.println("way steps not reached");
    // we failed, either left or right is wrong
    if (!force) {
      this->_homed = false;
      return false;
    }
    //this will eventually break the motor. Let's hope for the best
    Serial.println("force homing");
    this->rotate(std::optional(this->_way_steps), DoorPosition::OPEN, false);
  }
  Serial.println("door is homed");
  this->_homed = true;
  this->_position = std::optional(DoorPosition::OPEN);
  this->rotate(std::optional(_edge_distance), DoorPosition::CLOSE, false, 40);
  return true;
}

bool DoorActuator::open() {
  if (!this->_homed) {
    return false;
  }
  if (this->_position == DoorPosition::OPEN) {
    return true;
  }
  this->_position = std::nullopt;
  this->rotate(std::optional(this->_way_steps - _edge_distance), DoorPosition::OPEN, false, 40);
  this->_position = DoorPosition::OPEN;
  return true;
}

bool DoorActuator::close() {
  if (!this->_homed) {
    return false;
  }
  if (this->_position == DoorPosition::CLOSE) {
    return true;
  }
  this->_position = std::nullopt;
  this->rotate(std::optional(this->_way_steps - _edge_distance), DoorPosition::CLOSE, false);
  this->_position = DoorPosition::CLOSE;
  return true;
}
