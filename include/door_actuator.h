#pragma once
#include "TMCStepper.h"
#include <cstdint>
#include <optional>
#include <sys/types.h>

constexpr static auto EN_PIN{19};
constexpr static auto STEP_PIN{18};
constexpr static auto STALL_PIN{5};
constexpr static auto DRIVER_ADDRESS{0b00};
constexpr static auto R_SENSE{0.11f};

enum class DoorPosition {
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

static const char* state_name(DoorState state) {
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


class DoorActuator {

  public:


    DoorActuator(Stream* serial, uint8_t stall_thrs):
      _driver(serial, R_SENSE, DRIVER_ADDRESS),
      _stall_thrs{stall_thrs}
    {
      _mutex = xSemaphoreCreateMutex();
    };

    ~DoorActuator()
    {
      vSemaphoreDelete(_mutex);
    }

    bool setup();

    uint32_t rotate_infinite(const DoorPosition direction) {
      return this->rotate(std::nullopt, direction, true);
    }

    DoorState get_state() {
      return _state;
    }

    void notify_stalled();

    bool home(bool force = false);

    bool open();

    bool close();

    bool _stalled = false;
    DoorState _state = DoorState::UNKNOWN;
    const uint32_t _way_steps = 125000; //TODO: make this configureable
    const uint32_t _edge_distance = 10000; //TODO: make this configureable
    TMC2209Stepper _driver;
    uint8_t _stall_thrs;

    SemaphoreHandle_t _mutex = NULL;

  private:
    uint32_t rotate(
        std::optional<const uint32_t> steps,
        const DoorPosition direction,
        const bool stallguard = true,
        const uint32_t step_delay = 70
        );
};
