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

enum DoorPosition {
  OPEN,
  CLOSE
};


class DoorActuator {

  public:


    DoorActuator(Stream* serial, uint8_t stall_thrs):
      _driver(serial, R_SENSE, DRIVER_ADDRESS),
      _stall_thrs{stall_thrs}
    {};

    bool setup();

    uint32_t rotate_infinite(const DoorPosition direction) {
      return this->rotate(std::nullopt, direction, true);
    }


    void notify_stalled();

    bool home(bool force = false);

    bool open();

    bool close();

  private:
    uint32_t rotate(
        std::optional<const uint32_t> steps,
        const DoorPosition direction,
        const bool stallguard = true,
        const uint32_t step_delay = 70
        );
    TMC2209Stepper _driver;
    uint8_t _stall_thrs;
    const uint32_t _way_steps = 125000; //TODO: make this configureable
    const uint32_t _edge_distance = 10000; //TODO: make this configureable
    bool _stalled = false;
    bool _homed = false;
    std::optional<DoorPosition> _position = std::nullopt;
};
