#pragma once
#include "TMCStepper.h"
#include <cstdint>
#include <optional>

constexpr static auto EN_PIN{19};
constexpr static auto STEP_PIN{18};
constexpr static auto STALL_PIN{5};
constexpr static auto DRIVER_ADDRESS{0b00};
constexpr static auto R_SENSE{0.11f};

enum Direction {
  OPEN,
  CLOSE
};


class DoorActuator {

  public:


    DoorActuator(uint8_t stall_thrs, uint32_t delay_step):
      _driver((Stream*) &Serial2, R_SENSE, DRIVER_ADDRESS),
      _stall_thrs{stall_thrs},
      _delay_step{delay_step}
    {};

    bool setup();

    int rotate_infinite(const Direction direction) {
      return this->rotate(std::nullopt, direction, true);
    }

    int rotate(std::optional<const uint32_t> steps, const Direction direction, const bool stallguard = true);

    void notify_stalled();

  private:
    TMC2209Stepper _driver;
    uint8_t _stall_thrs;
    uint32_t _delay_step;
    bool _stalled = false;
};
