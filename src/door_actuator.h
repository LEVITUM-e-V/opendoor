#pragma once
#include "TMCStepper.h"
#include <cstdint>
#include <optional>
#include <sys/types.h>
#include <freertos/queue.h>

constexpr static auto EN_PIN{19};
constexpr static auto STEP_PIN{18};
constexpr static auto STALL_PIN{5};
constexpr static auto DRIVER_ADDRESS{0b00};
constexpr static auto R_SENSE{0.11f};

constexpr static auto MICROSTEPS{16};
constexpr static uint32_t FULL_STEPS_PER_ROTATION{200 * 15 / 4}; // 1.8 degree stepper + gear ratio
constexpr static uint32_t MAX_HOMING_STEPS{4 * MICROSTEPS * FULL_STEPS_PER_ROTATION};
constexpr static uint32_t STEPS_DISTANCE_OPEN_CLOSED{2 * MICROSTEPS * FULL_STEPS_PER_ROTATION};
constexpr static uint32_t HOMING_EDGE_DISTANCE_STEPS{2500};
constexpr static uint32_t HOMING_STEP_DELAY_MICROSECONDS{200};

constexpr static uint32_t DRIVER_CMD_DELAY_MS {500};

enum class DoorDirection {
  OPEN,
  CLOSE
};

enum class DoorState {
  UNKNOWN,
  HOMING,
  OPEN,
  CLOSING,
  CLOSED,
  OPENING
};

const char* state_name(DoorState state);

enum class DoorError {
  NOT_HOMED,
  ALREADY_CLOSED,
  ALREADY_OPEN,
  IN_MOTION,
  QUEUE_FULL,
};

const char* error_name(DoorError state);

class DoorActuator {
  public:
    DoorActuator(Stream* serial, uint8_t stall_thrs):
      _driver(serial, R_SENSE, DRIVER_ADDRESS),
      _stall_thrs{stall_thrs}
    {};

    ~DoorActuator()
    {};

    bool setup();
    void notify_stalled();
    std::optional<DoorError> home();
    std::optional<DoorError> open();
    std::optional<DoorError> close();
    DoorState get_state() {
      return _state;
    }

    uint32_t _position = 0;
    bool _homed = false;
    bool _stalled = false;
    DoorState _state = DoorState::UNKNOWN;
    TMC2209Stepper _driver;
    uint8_t _stall_thrs;
    QueueHandle_t _commandQueue;

  private:
    std::optional<DoorError> rotate(
        std::optional<const uint32_t> steps,
        const DoorDirection direction,
        const uint32_t step_delay = 200
        );
};
