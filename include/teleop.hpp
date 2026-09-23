#pragma once

#include "autons.hpp"

bool R1Mode = false;
bool R2Mode = false;
bool AMode = false;
bool YMode = false;

void updateButtons() {
    R1Mode = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    R2Mode = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    AMode = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    YMode = controller.get_digital(DIGITAL_Y);
}

void tankBasic() {
    int leftThrottle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightThrottle = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    if (std::abs(leftThrottle) < 5) leftThrottle = 0;
    if (std::abs(rightThrottle) < 5) rightThrottle = 0;

    drive.velocity(-rightThrottle * 5, -leftThrottle * 5);
}

void cascadeCode() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        autoLiftUntil = 0;
        autoDownUntil = 0;
        cascadeSpeed = -12000;
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        autoLiftUntil = 0;
        autoDownUntil = 0;
        cascadeSpeed = 12000;
    } else {
        cascadeSpeed = 0;
    }
    cascadeHold = true;
}

void pistonClawCode() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && !R1Mode) {
        claw.piston.set_value(true);
        hasClamped = true;
        autoLiftUntil = pros::millis() + 100;
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !R2Mode) {
        claw.piston.set_value(false);
        hasClamped = false;
    }
}
