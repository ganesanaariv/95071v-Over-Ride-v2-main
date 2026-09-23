#pragma once

// Use the fake PROS API while exercising the actual competition callbacks.
#define _PROS_MAIN_H_
#include "src/main.cpp"
#include <cassert>
#include <cstdlib>
#include <iostream>

template <typename Predicate> void waitFor(Predicate ready) {
    const uint32_t started = pros::millis();
    while (!ready() && pros::millis() - started < 1000)
        pros::delay(1);
    assert(ready());
}

Result motionResult() {
    std::lock_guard<pros::Mutex> lock(chassisMutex);
    return motion.result;
}

bool afterWait = false;

void earlyReturnExample() {
    setPos(72, 24, 0);
    setTargetPos(72, 48);
    autoLiftUntil = pros::millis() + 500;
    if (!untilTargetSettled(.75, 50)) return;
    afterWait = true;
}

void continueAfterWaitExample() {
    setPos(72, 24, 0);
    setTargetPos(72, 48);
    assert(!untilTargetPos(1, 50));
    assert(motionResult() == Result::Running);
    afterWait = true;
}

void testDirectAutons() {
    selectedAuton = earlyReturnExample;
    autonomous();
    assert(!afterWait);
    assert(fake.voltage[FL_PORT] == 0 && fake.voltage[CASCADE_PORT] == 0);
    assert(autoLiftUntil == 0 && !chassisTaskControl.enabled);

    selectedAuton = continueAfterWaitExample;
    autonomous();
    assert(afterWait);
    assert(fake.voltage[FL_PORT] == 0);

    selectedAuton = exampleIdle;
    fake.buttons[pros::E_CONTROLLER_DIGITAL_A] = true;
    startAuton();
    assert(cascadeTaskControl.enabled && !chassisTaskControl.enabled && !autoClampTaskControl.enabled);
    updateButtons();
    fake.buttons[pros::E_CONTROLLER_DIGITAL_A] = false;
    updateButtons();
}

void testChassisHelpers() {
    setPos(72, 24, 0);
    enableTask(chassisTaskControl);
    setTargetPos(72, 48);
    waitFor([] { return fake.voltage[FL_PORT] < 0; });
    pauseDrive();
    waitFor([] { return fake.voltage[FL_PORT] == 0; });
    setTargetPos(72, 36);
    pros::delay(30);
    assert(fake.voltage[FL_PORT] == 0);
    resumeDrive();
    waitFor([] { return fake.voltage[FL_PORT] < 0; });

    setMoveSpeedAtDistance(20, 0);
    waitFor([] { return maxMoveSpeed == 0; });
    waitFor([] { return fake.voltage[FL_PORT] == 0; });
    setMoveSpeed(200, 80);
    assert(maxMoveSpeed == 200 && minMoveSpeed == 80);
    setMoveSpeed(50, 100);
    assert(maxMoveSpeed == 50 && minMoveSpeed == 50);
    setMoveSpeed(200);
    assert(minMoveSpeed == 0);
    pros::delay(30);
    assert(maxMoveSpeed == 200 && fake.voltage[FL_PORT] < 0);

    setPos(72, 24, 0);
    setTargetPos(72, 24);
    assert(untilTargetSettled());
    waitFor([] { return motionResult() == Result::Settled; });
    assert(fake.voltage[FL_PORT] == 0);

    MotionOptions options;
    options.timeout = 50;
    setTargetPos(72, 48, options);
    assert(!untilTargetPos(1, 500));
    assert(motionResult() == Result::TimedOut);

    setTargetPos(72, 48);
    fake.sensorValid = false;
    waitFor([] { return motionResult() == Result::SensorFault; });
    assert(fake.voltage[FL_PORT] == 0);
    fake.sensorValid = true;
    setPos(72, 24, 90);
    assert(resetFromWall(WallSensor::Front, FieldWall::Bottom) == Result::Rejected);
    assert(getRobotPose().x == 72 && getRobotPose().y == 24 && driveDisabled);

    setPos(72, 24, 0);
    const int readings = fake.wallReadCount;
    const uint32_t resetStarted = pros::millis();
    assert(resetFromWall(WallSensor::Front, FieldWall::Bottom, 150) == Result::Applied);
    assert(pros::millis() - resetStarted < 150);
    assert(fake.wallReadCount == readings + 1);
    assert(driveDisabled && fake.voltage[FL_PORT] == 0);

    const Pose beforeFailedReset = getRobotPose();
    fake.sensorValid = false;
    const uint32_t failedResetStarted = pros::millis();
    assert(resetFromWall(WallSensor::Front, FieldWall::Bottom, 150) == Result::TimedOut);
    assert(pros::millis() - failedResetStarted < 200);
    fake.sensorValid = true;
    pros::delay(30);
    assert(fake.wallReadCount == readings + 1);
    assert(getRobotPose().x == beforeFailedReset.x && getRobotPose().y == beforeFailedReset.y);

    disableTask(chassisTaskControl);
    drive.stop();
    fake.encoderPosition = -10000;
    setPos(72, 24, 0);
    resumeDrive();
    enableTask(chassisTaskControl);
    pros::delay(30);
    assert(getRobotPose().x == 72 && getRobotPose().y == 24);

    // Slow controller I/O runs independently of chassis sampling.
    fake.printDelay = 60;
    const int prints = fake.printCount;
    waitFor([&] { return fake.printCount > prints; });
    const int samples = fake.samples;
    pros::delay(50);
    assert(fake.samples - samples >= 8);
    fake.printDelay = 0;
    disableTask(chassisTaskControl);
    drive.stop();
}

void testMechanismsAndDriver() {
    enableTask(cascadeTaskControl);
    cascadeSpeed = 3400;
    waitFor([] { return fake.voltage[CASCADE_PORT] == 3400; });
    autoLiftUpSpeed = 7200;
    autoLiftUntil = pros::millis() + 80;
    waitFor([] { return fake.voltage[CASCADE_PORT] == 7200; });
    waitFor([] { return fake.voltage[CASCADE_PORT] == 3400; });
    cascadeSpeed = 0;
    autoLiftDownSpeed = 6000;
    autoDownUntil = pros::millis() + 80;
    waitFor([] { return fake.voltage[CASCADE_PORT] == -6000; });
    waitFor([] { return fake.voltage[CASCADE_PORT] == 0; });

    enableTask(autoClampTaskControl);
    shouldClamp = true;
    fake.acquisitionDistance = 0;
    pros::delay(30);
    assert(!hasClamped && !fake.piston['A']);
    fake.acquisitionDistance = 60;
    assert(untilClamped(300));
    assert(fake.piston['A']);
    waitFor([] { return fake.voltage[CASCADE_PORT] == 7200; });
    shouldClamp = false;
    waitFor([] { return !hasClamped.load(); });
    assert(!fake.piston['A']);
    disableTask(autoClampTaskControl);

    fake.buttons[pros::E_CONTROLLER_DIGITAL_L2] = true;
    cascadeCode();
    waitFor([] { return fake.voltage[CASCADE_PORT] == -12000; });
    assert(autoLiftUntil == 0 && autoDownUntil == 0);
    fake.buttons[pros::E_CONTROLLER_DIGITAL_L2] = false;
    cascadeCode();
    waitFor([] { return fake.voltage[CASCADE_PORT] == 0; });

    fake.sticks[pros::E_CONTROLLER_ANALOG_LEFT_Y] = 50;
    fake.sticks[pros::E_CONTROLLER_ANALOG_RIGHT_Y] = 100;
    tankBasic();
    assert(fake.rpm[FL_PORT] == 500 && fake.rpm[FR_PORT] == 250);
    assert(fake.rpm[ML_PORT] == 166 && fake.rpm[MR_PORT] == 83);

    fake.buttons[pros::E_CONTROLLER_DIGITAL_R1] = true;
    pistonClawCode();
    updateButtons();
    assert(fake.piston['A']);
    waitFor([] { return fake.voltage[CASCADE_PORT] > 0; });
    fake.disabled = true;
    disabled();
    pros::delay(30);
    assert(fake.voltage[CASCADE_PORT] == 0 && fake.voltage[FL_PORT] == 0);
    assert(autoLiftUntil == 0 && autoDownUntil == 0);
    assert(fake.piston['A']);
    assert(!chassisTaskControl.enabled && !cascadeTaskControl.enabled && !autoClampTaskControl.enabled);
}

#if ENABLE_PID_TUNER
#include "tests/pid_tuner_tests.hpp"
#endif

int main() {
    initialize();
    testDirectAutons();
    testChassisHelpers();
#if ENABLE_PID_TUNER
    testPidTuner();
    testPidTunerFaults();
#endif
    testMechanismsAndDriver();
    std::cout << "Task, mechanism, and direct autonomous tests passed" << std::endl;
    std::_Exit(0);
}
