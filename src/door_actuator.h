#pragma once
#include <Arduino.h>
#include <TMCStepper.h>
#include <cstdint>


class DoorActuator {

  public:
    bool setup();

    int rotate_infinite(const uint32_t steps);

};
