#pragma once

#include "api.h"

class Claw {
public:
    pros::adi::DigitalOut piston;
    pros::Distance distance;

    Claw(uint8_t pistonPort, uint8_t distancePort) : piston(pistonPort), distance(distancePort) {}
};
