#include <Arduino.h>
#include <TMCStepper.h>
#include <cstdint>
#include <optional>

#include "door_actuator.h"

#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "freertos/queue.h"

#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(MotionCommand)

struct MotionCommand {
  bool homing_before_cmd;
  DoorActuator* door;
  DoorDirection direction;
  uint32_t steps;
  uint32_t step_delay;
};


void DoorActuator::notify_stalled() {
  _stalled = true;
}

const char* state_name(DoorState state) {
  switch (state) {
    case DoorState::UNKNOWN:
      return "unknown/failed";
    case DoorState::HOMING:
      return "homing";
    case DoorState::OPEN:
      return "open";
    case DoorState::CLOSING:
      return "closing";
    case DoorState::CLOSED:
      return "closed";
    case DoorState::OPENING:
      return "opening";
    default:
      return "unknown state";
  }
}

const char* error_name(DoorError state) {
  switch (state) {
    case DoorError::NOT_HOMED:
      return "the door is not homed";
    case DoorError::ALREADY_CLOSED:
      return "the door is already closed";
    case DoorError::ALREADY_OPEN:
      return "the door is already open";
    case DoorError::IN_MOTION:
      return "the door is currently in motion";
    case DoorError::QUEUE_FULL:
      return "queue full";
    default:
      return "unknown error";
  }
}

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

  DoorActuator* actuator = (DoorActuator*) arg;
  MotionCommand cmd;

  while (true) {
    if (xQueueReceive(
          actuator->_commandQueue,
          &cmd,
          portMAX_DELAY
          )) {
      uint32_t step_counter = 0;
      actuator->_stalled = false;

      if (cmd.homing_before_cmd) {
          actuator->_driver.shaft(false);
          delay(DRIVER_CMD_DELAY_MS);
          digitalWrite(EN_PIN, LOW);
          delay(DRIVER_CMD_DELAY_MS);
          while (!actuator->_stalled && step_counter < MAX_HOMING_STEPS) {
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(HOMING_STEP_DELAY_MICROSECONDS);
            digitalWrite(STEP_PIN, LOW);
            delayMicroseconds(HOMING_STEP_DELAY_MICROSECONDS);
            feedTheDog();
            ++step_counter;
          }
          digitalWrite(EN_PIN, HIGH);
          step_counter = 0;
          if (actuator->_stalled) {
            Serial.println("stall detected");
            actuator->_driver.shaft(true);
            delay(DRIVER_CMD_DELAY_MS);
            digitalWrite(EN_PIN, LOW);
            delay(DRIVER_CMD_DELAY_MS);
            for (int i = 0; i < HOMING_EDGE_DISTANCE_STEPS; ++i) {
              digitalWrite(STEP_PIN, HIGH);
              delayMicroseconds(HOMING_STEP_DELAY_MICROSECONDS);
              digitalWrite(STEP_PIN, LOW);
              delayMicroseconds(HOMING_STEP_DELAY_MICROSECONDS);
              feedTheDog();
            }
            digitalWrite(EN_PIN, HIGH);
            actuator->_position = HOMING_EDGE_DISTANCE_STEPS;
            actuator->_state = DoorState::OPEN;
            Serial.println("door is homed");
          } else {
            actuator->_state = DoorState::UNKNOWN;
            Serial.println("Homing failed: no stall detected");
          }
      }

      actuator->_driver.shaft(cmd.direction != DoorDirection::OPEN);
      delay(DRIVER_CMD_DELAY_MS);
      digitalWrite(EN_PIN, LOW);
      delay(DRIVER_CMD_DELAY_MS);
      while (step_counter < cmd.steps) {
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(cmd.step_delay);
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(cmd.step_delay);
          feedTheDog();
          ++step_counter;
      }
      digitalWrite(EN_PIN, HIGH);

      if (cmd.steps > 0) {
        if (cmd.direction == DoorDirection::OPEN) {
          actuator->_state = DoorState::OPEN;
          actuator->_position -= step_counter;
        } else if (cmd.direction == DoorDirection::CLOSE) {
          actuator->_state = DoorState::CLOSED;
          actuator->_position += step_counter;
        }
      }
    }
  }
}

bool DoorActuator::setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  digitalWrite(EN_PIN, HIGH);


  _driver.begin();
  _driver.toff(4);
  _driver.blank_time(24);
  _driver.I_scale_analog(false);
  _driver.internal_Rsense(false); 
  _driver.mstep_reg_select(true);
  _driver.rms_current(400);
  _driver.microsteps(MICROSTEPS);

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

  _commandQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);

  if (xTaskCreatePinnedToCore(
      rotate_task, "RotateTask", 4096, this, 3, NULL, 1) != pdPASS) {
    Serial.println("Could not create rotate task");
    return false;
  }

  return true;
}

std::optional<DoorError>  DoorActuator::rotate(
    std::optional<const uint32_t> steps,
    const DoorDirection direction,
    const uint32_t step_delay
    ) {


  this->_state = direction == DoorDirection::OPEN ? DoorState::OPENING : DoorState::CLOSING;

   MotionCommand command = {
      .homing_before_cmd = false,
      .door = this,
      .direction = direction,
      .steps = steps.value_or(0),
      .step_delay = step_delay,
  };

  if (xQueueSendToBack(_commandQueue, &command, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println("Queue full, command not sent");
    return DoorError::QUEUE_FULL;
  }
  return {};
}

std::optional<DoorError> DoorActuator::home() {
  MotionCommand command = {
      .homing_before_cmd = true,
      .door = this,
      .direction = DoorDirection::OPEN,
      .steps = 0,
      .step_delay = 300,
  };

  if (xQueueSendToBack(_commandQueue, &command, pdMS_TO_TICKS(100)) != pdPASS) {
    return DoorError::QUEUE_FULL;
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
  this->rotate(std::optional(STEPS_DISTANCE_OPEN_CLOSED), DoorDirection::OPEN, 120);
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
  this->rotate(std::optional(STEPS_DISTANCE_OPEN_CLOSED), DoorDirection::CLOSE);
  return {};
}
