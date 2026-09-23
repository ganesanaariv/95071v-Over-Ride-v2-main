#pragma once

void pressTunerButton(pros::controller_digital_e_t button) {
    fake.buttons[button] = true;
    waitFor([&] { return fake.previousButtons[button].load(); });
    fake.buttons[button] = false;
    waitFor([&] { return !fake.previousButtons[button].load(); });
}

MotionRequest tunerRequest() {
    std::lock_guard<pros::Mutex> lock(chassisMutex);
    return motion.request();
}

Gains tunerGains(unsigned loop) {
    std::lock_guard<pros::Mutex> lock(chassisMutex);
    if (loop == 0) return motion.translation.gains;
    if (loop == 1) return motion.turning.gains;
    return motion.drivingHeading.gains;
}

void waitForTuner() {
    waitFor([] { return chassisTaskControl.enabled && !printTaskControl.enabled; });
    waitFor([] {
        std::lock_guard<std::mutex> lock(fake.screenMutex);
        return fake.screen[0] == "Drive M250 A:go" && fake.screen[2] == "X:loop B:exit  ";
    });
    assert(!cascadeTaskControl.enabled && !autoClampTaskControl.enabled);
    assert(fake.voltage[CASCADE_PORT] == 0);
}

void fakeTunerTravel(double distance) {
    const double start = fake.encoderPosition;
    for (unsigned i = 1; i <= 30; ++i) {
        fake.encoderPosition = start - distance * i / 30 / config.odometry.inchesPerTick;
        pros::delay(20);
    }
}

void testPidTuner() {
    setMoveSpeed(390, 40);
    setTurnSpeed(410);
    maxMotorSpeed = 450;
    const Gains original = tunerGains(0);
    autoLiftUntil = pros::millis() + 1000;
    std::thread tuner(runPidTuner);
    waitForTuner();
    assert(autoLiftUntil == 0 && minMoveSpeed == 0 && maxMotorSpeed == 600);

    pressTunerButton(pros::E_CONTROLLER_DIGITAL_L1);
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_R1);
    fake.buttons[pros::E_CONTROLLER_DIGITAL_LEFT] = true;
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_R1);
    fake.buttons[pros::E_CONTROLLER_DIGITAL_LEFT] = false;
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_UP);
    assert(std::fabs(tunerGains(0).p - original.p - PID_P_STEP) < 1e-9);
    assert(std::fabs(tunerGains(0).d - original.d - PID_D_STEP * 1.1) < 1e-9);
    assert(config.motion.translation.p == original.p && config.motion.translation.d == original.d);
    assert(maxMoveSpeed == 275 && minMoveSpeed == 0);

    pressTunerButton(pros::E_CONTROLLER_DIGITAL_A);
    waitFor([] { return motionResult() == Result::Running && fake.voltage[FL_PORT] < 0; });
    assert(tunerRequest().kind == MotionKind::Distance && tunerRequest().distance == 24);
    assert(tunerRequest().options.timeout == 0);
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_L1);
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_X);
    assert(std::fabs(tunerGains(0).p - original.p - PID_P_STEP) < 1e-9);

    // Overshooting must not release the next move; arrival also requires low speed and dwell.
    fakeTunerTravel(25);
    pros::delay(150);
    assert(motionResult() == Result::Running && tunerRequest().distance == 24);
    fakeTunerTravel(-1);
    assert(motionResult() == Result::Running);
    waitFor([] { return motionResult() == Result::Settled; });
    pros::delay(150);
    assert(tunerRequest().distance == 24 && fake.voltage[FL_PORT] == 0);
    waitFor([] { return tunerRequest().distance == -24 && motionResult() == Result::Running; });
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_A);
    fakeTunerTravel(-24);
    waitFor([] { return motionResult() == Result::Settled; });
    pros::delay(PID_TEST_PAUSE_MS + 100);
    assert(tunerRequest().distance == -24 && motionResult() == Result::Settled);

    pressTunerButton(pros::E_CONTROLLER_DIGITAL_X);
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_R1);
    assert(std::fabs(tunerGains(1).d - config.motion.turn.d - PID_D_STEP) < 1e-9);
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_A);
    assert(tunerRequest().kind == MotionKind::Turn);
    const double initialHeading = getRobotPose().heading;
    assert(std::fabs(tunerRequest().target.heading - initialHeading - 90) < 1e-9);
    for (unsigned i = 1; i <= 30; ++i) {
        fake.rotation = i * 3;
        pros::delay(20);
    }
    waitFor([] { return motionResult() == Result::Settled; });
    waitFor([&] { return std::fabs(tunerRequest().target.heading - initialHeading) < 1e-9; });
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_A);
    for (unsigned i = 1; i <= 30; ++i) {
        fake.rotation = 90 - i * 3;
        pros::delay(20);
    }
    waitFor([] { return motionResult() == Result::Settled; });
    pros::delay(50);

    pressTunerButton(pros::E_CONTROLLER_DIGITAL_X);
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_L2);
    assert(std::fabs(tunerGains(2).p - config.motion.drivingHeading.p + PID_P_STEP) < 1e-9);
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_A);
    assert(tunerRequest().kind == MotionKind::Distance && tunerRequest().distance == 24);
    waitFor([] { return fake.voltage[FL_PORT] < 0; });
    fake.printDelay = 60;
    const int prints = fake.printCount;
    waitFor([&] { return fake.printCount > prints; });
    const int samples = fake.samples;
    pros::delay(50);
    assert(fake.samples - samples >= 8);
    fake.printDelay = 0;

    fake.buttons[pros::E_CONTROLLER_DIGITAL_B] = true;
    tuner.join();
    fake.buttons[pros::E_CONTROLLER_DIGITAL_B] = false;
    assert(!chassisTaskControl.enabled && cascadeTaskControl.enabled && printTaskControl.enabled);
    assert(fake.voltage[FL_PORT] == 0 && fake.voltage[FR_PORT] == 0);
    assert(maxMoveSpeed == 390 && minMoveSpeed == 40 && maxTurnSpeed == 410 && maxMotorSpeed == 450);
    assert(std::fabs(tunerGains(0).p - original.p - PID_P_STEP) < 1e-9);
}

void testPidTunerFaults() {
    std::thread tuner(runPidTuner);
    waitForTuner();
    pressTunerButton(pros::E_CONTROLLER_DIGITAL_A);
    fake.sensorValid = false;
    waitFor([] { return motionResult() == Result::SensorFault; });
    pros::delay(PID_TEST_PAUSE_MS + 100);
    assert(motionResult() == Result::SensorFault && fake.voltage[FL_PORT] == 0);
    fake.buttons[pros::E_CONTROLLER_DIGITAL_B] = true;
    tuner.join();
    fake.buttons[pros::E_CONTROLLER_DIGITAL_B] = false;
    fake.sensorValid = true;

    // Leaving driver mode or losing the controller exits without a new motion.
    for (auto* exitFlag : {&fake.disabled, &fake.autonomous, &fake.connected}) {
        std::thread interrupted(runPidTuner);
        waitForTuner();
        pressTunerButton(pros::E_CONTROLLER_DIGITAL_A);
        waitFor([] { return fake.voltage[FL_PORT] < 0; });
        *exitFlag = !exitFlag->load();
        interrupted.join();
        assert(!chassisTaskControl.enabled && fake.voltage[FL_PORT] == 0);
        if (fake.disabled || fake.autonomous) assert(!cascadeTaskControl.enabled && !printTaskControl.enabled);
        *exitFlag = !exitFlag->load();
    }
    std::lock_guard<pros::Mutex> lock(chassisMutex);
    motion.translation.gains = config.motion.translation;
    motion.turning.gains = config.motion.turn;
    motion.drivingHeading.gains = config.motion.drivingHeading;
}
