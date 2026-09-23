#pragma once

#include "api.h"
#include <algorithm>

class Cascade {
    pros::Motor cascadeLeft;
    pros::Motor cascadeRight;

public:
    Cascade(int8_t leftPort, int8_t rightPort)
        : cascadeLeft(leftPort, pros::v5::MotorGears::blue), cascadeRight(rightPort, pros::v5::MotorGears::blue) {}

    void initialize() {
        cascadeLeft.set_voltage_limit(12000);
        cascadeRight.set_voltage_limit(12000);
        cascadeLeft.set_current_limit(2500);
        cascadeRight.set_current_limit(2500);
    }

    void moveCascadeVoltage(int voltage) {
        voltage = std::clamp(voltage, -12000, 12000);
        cascadeLeft.move_voltage(voltage);
        cascadeRight.move_voltage(voltage);
    }

    void hold() {
        cascadeLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        cascadeRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    void coast() {
        cascadeLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        cascadeRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
};
