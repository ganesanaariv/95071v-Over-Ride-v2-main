#pragma once

#include "auton/auton.hpp"
#include <cstdio>

const double PID_TEST_DISTANCE_IN = 24;
const double PID_TEST_TURN_DEG = 90;
const double PID_TEST_POWER = 250;
const uint32_t PID_TEST_PAUSE_MS = 750;
const uint32_t PID_TEST_MOTION_TIMEOUT_MS = 4000;
const double PID_P_STEP = 0.5;
const double PID_D_STEP = 0.01;

// Driver-only: A repeats/stops after this move; B cancels and exits; X selects the loop.
// L1/L2: P +/-; R1/R2: D +/-; Left: fine (0.1x); Right: coarse (10x); Up/Down: power +/-25.
// Gains stay in RAM. Copy the printed values into config.hpp to keep them after reboot.
void runPidTuner() {
    const double savedMove = maxMoveSpeed.load(), savedMinimum = minMoveSpeed.load();
    const double savedTurn = maxTurnSpeed.load(), savedMotor = maxMotorSpeed.load();
    const bool savedPause = driveDisabled.load();
    disableTask(chassisTaskControl);
    drive.stop();
    disableTask(cascadeTaskControl);
    disableTask(autoClampTaskControl);
    cascadeSpeed = 0;
    autoLiftUntil = 0;
    autoDownUntil = 0;
    cascade.moveCascadeVoltage(0);
    disableTask(printTaskControl);
    controller.print(0, 0, "PID tuner starting");

    const Pose pose = getRobotPose();
    setPos(pose.x, pose.y, pose.heading);
    drive.brake(true);
    setMoveSpeed(PID_TEST_POWER, 0);
    setTurnSpeed(PID_TEST_POWER);
    maxMotorSpeed = 600;
    resumeDrive();
    enableTask(chassisTaskControl);

    const pros::controller_digital_e_t buttons[] = {
        pros::E_CONTROLLER_DIGITAL_A,  pros::E_CONTROLLER_DIGITAL_X,    pros::E_CONTROLLER_DIGITAL_L1,
        pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_R1,   pros::E_CONTROLLER_DIGITAL_R2,
        pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN,
    };
    for (auto button : buttons)
        controller.get_digital_new_press(button);

    Gains* gains[] = {&motion.translation.gains, &motion.turning.gains, &motion.drivingHeading.gains};
    const char* names[] = {"Drive", "Turn", "Hold"};
    const char* configNames[] = {"translation", "turn", "drivingHeading"};
    unsigned selected = 0, line = 0;
    bool repeating = false, moving = false, outbound = true;
    double power = PID_TEST_POWER, heading = pose.heading;
    double error = 0, seconds = 0;
    Result result = Result::Idle;
    uint32_t started = 0, nextMoveAt = 0, lastPrint = pros::millis() - 100;
    std::printf("PID tuner: L1/L2 P, R1/R2 D, Left fine, Right coarse, Up/Down power, X loop, A repeat, B exit\n");

    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) == 1) {
            std::lock_guard<pros::Mutex> lock(chassisMutex);
            motion.stop(Result::Idle);
            driveOutput = {};
            break;
        }
        bool toggleRepeat = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) == 1;
        bool changeLoop = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) == 1;
        bool increaseP = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) == 1;
        bool decreaseP = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) == 1;
        bool increaseD = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) == 1;
        bool decreaseD = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) == 1;
        bool increasePower = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) == 1;
        bool decreasePower = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) == 1;
        uint32_t now = pros::millis();

        if (changeLoop) {
            selected = (selected + 1) % 3;
            result = Result::Idle;
            error = seconds = 0;
        }
        double step = 1;
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) == 1) step = 0.1;
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == 1) step = 10;
        if (increaseP || decreaseP || increaseD || decreaseD) {
            Gains updated;
            {
                std::lock_guard<pros::Mutex> lock(chassisMutex);
                Gains& g = *gains[selected];
                g.p = std::clamp(g.p + (increaseP - decreaseP) * PID_P_STEP * step, 0.0, 999.99);
                g.d = std::clamp(g.d + (increaseD - decreaseD) * PID_D_STEP * step, 0.0, 99.999);
                updated = g;
            }
            std::printf("%s{%.2f, %.3f}\n", configNames[selected], updated.p, updated.d);
        }
        power = std::clamp(power + (increasePower - decreasePower) * 25, 25.0, 600.0);
        setMoveSpeed(power, 0);
        setTurnSpeed(power);

        if (toggleRepeat) {
            repeating = !repeating;
            if (repeating && !moving) {
                outbound = true;
                heading = getRobotPose().heading;
                nextMoveAt = now;
            }
        }
        if (repeating && !moving && due(now, nextMoveAt)) {
            MotionOptions options;
            options.timeout = PID_TEST_MOTION_TIMEOUT_MS;
            if (selected == 1)
                setTargetHeading(heading + (outbound ? PID_TEST_TURN_DEG : 0), options);
            else
                driveDistance(outbound ? PID_TEST_DISTANCE_IN : -PID_TEST_DISTANCE_IN, heading, options);
            moving = true;
            started = now;
        }

        Gains displayed;
        {
            std::lock_guard<pros::Mutex> lock(chassisMutex);
            displayed = *gains[selected];
            if (moving) {
                result = motion.result;
                error = selected == 0 ? motion.distanceRemaining(odometry.estimate()) : motion.headingError;
                seconds = (now - started) / 1000.0;
            }
        }
        if (moving && result != Result::Running) {
            moving = false;
            std::printf("PID %s %s: %s, %.3fs, error %.3f%s\n", names[selected], outbound ? "out" : "back",
                        resultName(result), seconds, error, selected == 0 ? "in" : "deg");
            if (result == Result::Settled) {
                outbound = !outbound;
                nextMoveAt = now + PID_TEST_PAUSE_MS;
            } else {
                repeating = false;
            }
        }

        if (elapsed(now, lastPrint, 100)) {
            char text[64];
            if (line == 0) {
                if (moving)
                    std::snprintf(text, sizeof(text), "%s %s %s", names[selected], outbound ? ">" : "<",
                                  repeating ? "run" : "last");
                else if (result != Result::Idle && result != Result::Settled)
                    std::snprintf(text, sizeof(text), "%s", resultName(result));
                else if (repeating)
                    std::snprintf(text, sizeof(text), "%s settled", names[selected]);
                else
                    std::snprintf(text, sizeof(text), "%s M%3.0f A:go", names[selected], power);
            } else if (line == 1) {
                std::snprintf(text, sizeof(text), "P%.2f D%.3f", displayed.p, displayed.d);
            } else if (result == Result::Idle) {
                std::snprintf(text, sizeof(text), "X:loop B:exit");
            } else {
                std::snprintf(text, sizeof(text), "e%+.2f t%.2f", error, seconds);
            }
            controller.print(line, 0, "%-15.15s", text);
            line = (line + 1) % 3;
            lastPrint = now;
        }
        pros::delay(20);
    }

    disableTask(chassisTaskControl);
    drive.stop();
    setMoveSpeed(savedMove, savedMinimum);
    setTurnSpeed(savedTurn);
    maxMotorSpeed = savedMotor;
    driveDisabled = savedPause;
    for (unsigned i = 0; i < 3; ++i)
        std::printf("%s{%.2f, %.3f}\n", configNames[i], gains[i]->p, gains[i]->d);
    drive.brake(false);
    if (!pros::competition::is_disabled() && !pros::competition::is_autonomous()) {
        enableTask(cascadeTaskControl);
        enableTask(printTaskControl);
    }
}
