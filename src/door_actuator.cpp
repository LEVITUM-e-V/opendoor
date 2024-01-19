#include <Arduino.h>
#include <TMCStepper.h>
#include <cstdint>
#include <optional>

#include "door_actuator.h"

#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

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

struct RotateTaskParameter {
  bool stallguard;
  DoorActuator* door;
  DoorPosition direction;
  uint32_t steps;
  uint32_t step_delay;
};

void feedTheDog(){
  // feed dog 0
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG0.wdt_feed=1;                       // feed dog
  TIMERG0.wdt_wprotect=0;                   // write protect
  // feed dog 1
  TIMERG1.wdt_wprotect=TIMG_WDT_WKEY_VALUE; // write enable
  TIMERG1.wdt_feed=1;                       // feed dog
  TIMERG1.wdt_wprotect=0;                   // write protect
}

void rotate_task(void* arg) {

  RotateTaskParameter* params = (RotateTaskParameter*) arg;
  uint32_t step_counter = 0;
  digitalWrite(EN_PIN, LOW);
  params->door->_stalled = false;
  params->door->_driver.shaft(params->direction != DoorPosition::OPEN);
  params->door->_driver.rms_current(400);
  params->door->_driver.TPWMTHRS(0);
  params->door->_driver.SGTHRS(params->door->_stall_thrs);
  while (!(params->stallguard && params->door->_stalled)) {
     if (params->steps != 0 && step_counter >= params->steps) {
       break;
     }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(params->step_delay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(params->step_delay);
    feedTheDog();

    ++step_counter;
  }
  digitalWrite(EN_PIN, HIGH);
  params->door->_state = params->direction == DoorPosition::OPEN ? DoorState::OPEN : DoorState::CLOSED;
  free(params);
  vTaskDelete(NULL);
}

void homing_task(void* arg) {

  RotateTaskParameter* params = (RotateTaskParameter*) arg;
  digitalWrite(EN_PIN, LOW);
  params->door->_stalled = false;
  params->door->_driver.shaft(false); // open
  params->door->_driver.rms_current(400);
  params->door->_driver.TPWMTHRS(0);
  params->door->_driver.SGTHRS(params->door->_stall_thrs);

  uint32_t step_counter = 0;

  while (!params->door->_stalled) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(params->step_delay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(params->step_delay);
    feedTheDog();
    ++step_counter;
  }

  Serial.println("door is homed");
  params->door->_state = DoorState::OPEN;
  params->door->_driver.shaft(true);
  for (int i = 0; i < params->door->_edge_distance; ++i) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(40);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(40);
    feedTheDog();
  }

  digitalWrite(EN_PIN, HIGH);
  free(params);
  vTaskDelete(NULL);
}

uint32_t DoorActuator::rotate(
    std::optional<const uint32_t> steps,
    const DoorPosition direction,
    const bool stallguard,
    const uint32_t step_delay
    ) {


  this->_state = direction == DoorPosition::OPEN ? DoorState::OPENING : DoorState::CLOSING;
  RotateTaskParameter* params = (RotateTaskParameter*) malloc(sizeof(RotateTaskParameter));
  if (params == NULL) {
    Serial.println("malloc error");
    return 0;
  }

  params->stallguard = stallguard;
  params->door = this;
  params->steps = steps.value_or(0);
  params->step_delay = step_delay;
  params->direction = direction;

  if (xTaskCreatePinnedToCore(
      rotate_task,
      "RotateTask",
      2000,
      params,
      3,
      NULL,
      1) != pdPASS) {
    this->_state = DoorState::UNKNOWN;
    Serial.println("could not create rotate task");
    return 1;
  }
  return 0;
}

std::optional<DoorError> DoorActuator::home(bool force) {
  RotateTaskParameter* params = (RotateTaskParameter*) malloc(sizeof(RotateTaskParameter));
  if (params == NULL) {
    return DoorError::MALLOC_ERROR;
  }


  params->door = this;
  params->step_delay = 70;

  if (xTaskCreatePinnedToCore(
      homing_task,
      "HomeTask",
      2000,
      params,
      3,
      NULL,
      1) != pdPASS) {
    this->_state = DoorState::UNKNOWN;
    return DoorError::RTOS_TASK;
  }

  return {};
}

std::optional<DoorError> DoorActuator::open() {
  if (this->_state == DoorState::UNKNOWN) {
    return DoorError::NOT_HOMED;
  }
  if (this->_state == DoorState::CLOSING 
      || this->_state == DoorState::OPENING) {
    return DoorError::IN_MOTION;
  }
  if (this->_state == DoorState::OPEN) {
    return DoorError::ALREADY_OPEN;
  }
  this->rotate(std::optional(this->_way_steps - _edge_distance), DoorPosition::OPEN, false, 40);
  return {};
}

std::optional<DoorError> DoorActuator::close() {
  if (this->_state == DoorState::UNKNOWN) {
    return DoorError::NOT_HOMED;
  }
  if (this->_state == DoorState::CLOSING 
      || this->_state == DoorState::OPENING) {
    return DoorError::IN_MOTION;
  }
  if (this->_state == DoorState::CLOSED) {
    return DoorError::ALREADY_CLOSED;
  }
  this->rotate(std::optional(this->_way_steps - _edge_distance), DoorPosition::CLOSE, false);
  return {};
}
