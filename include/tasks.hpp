#pragma once

#include "setup.hpp"
#include "auton/localization.hpp"
#include "auton/motion.hpp"
#include <atomic>
#include <mutex>

struct BackgroundTaskControl {
    std::atomic<bool> enabled{false};
    std::atomic<bool> active{false};
};

BackgroundTaskControl chassisTaskControl;
BackgroundTaskControl odometryTaskControl;
BackgroundTaskControl cascadeTaskControl;
BackgroundTaskControl autoClampTaskControl;
BackgroundTaskControl printTaskControl;

pros::Task* chassis_task = nullptr;
pros::Task* odometry_task = nullptr;
pros::Task* cascade_task = nullptr;
pros::Task* clamp_task = nullptr;
pros::Task* print_task = nullptr;

std::atomic<int> cascadeSpeed{0};
std::atomic<bool> cascadeHold{true};
std::atomic<int> autoLiftUpSpeed{12000};
std::atomic<int> autoLiftDownSpeed{12000};
std::atomic<uint32_t> autoLiftUntil{0};
std::atomic<uint32_t> autoDownUntil{0};
std::atomic<bool> shouldClamp{false};
std::atomic<bool> hasClamped{false};
std::atomic<bool> goUpWhenClamped{true};

std::atomic<double> maxMoveSpeed{600};
std::atomic<double> minMoveSpeed{0};
std::atomic<double> maxTurnSpeed{600};
std::atomic<double> maxMotorSpeed{600};
std::atomic<Direction> driveMode{Direction::Auto};
std::atomic<bool> driveDisabled{false};

pros::Mutex chassisMutex;
Odometry odometry;
Motion motion;
Output driveOutput;
double desiredHeading = unspecified;
double speedChangeDistance = unspecified;
double speedChangePower = 0;
Vec speedChangePreviousPosition{};
uint32_t maximumCycleUs = 0;
std::atomic<uint32_t> missedCycles{0};

bool beginTaskCycle(BackgroundTaskControl& task) {
    if (!task.enabled.load()) return false;
    task.active = true;
    if (!task.enabled.load()) {
        task.active = false;
        return false;
    }
    return true;
}

void finishTaskCycle(BackgroundTaskControl& task) {
    task.active = false;
}

void enableTask(BackgroundTaskControl& task) {
    task.enabled = true;
}

void disableTask(BackgroundTaskControl& task) {
    task.enabled = false;
    while (task.active.load())
        pros::delay(1);
}

void odometryTask() {
    uint32_t nextTick = pros::millis();
    bool rebase = true;

    while (true) {
        if (!beginTaskCycle(odometryTaskControl)) {
            rebase = true;
            pros::delay(config.periodMs);
            continue;
        }

        const uint32_t now = pros::millis();
        {
            std::lock_guard<pros::Mutex> lock(chassisMutex);
            if (rebase) {
                odometry.rebase();
                nextTick = now;
                rebase = false;
            }
            odometry.update(drive.sample(), now);
        }
        finishTaskCycle(odometryTaskControl);

        nextTick += config.periodMs;
        const uint32_t finished = pros::millis();
        if (due(finished, nextTick)) nextTick = finished + config.periodMs;
        pros::delay(nextTick - finished);
    }
}

void chassisTask() {
    uint32_t nextTick = pros::millis();
    uint32_t previousTick = nextTick;
    bool resetHistory = true;

    while (true) {
        if (!beginTaskCycle(chassisTaskControl)) {
            resetHistory = true;
            pros::delay(config.periodMs);
            continue;
        }

        const uint64_t started = pros::micros();
        const uint32_t now = pros::millis();
        {
            std::lock_guard<pros::Mutex> lock(chassisMutex);
            if (resetHistory) {
                motion.resetHistory();
                previousTick = now - config.periodMs;
                nextTick = now;
                resetHistory = false;
            }
            const double dt = (now - previousTick) / 1000.0;
            previousTick = now;
            const Estimate& estimate = odometry.estimate();
            const bool valid = estimate.valid && !elapsed(now, estimate.sampledAt, config.staleMs + 1);
            if (!valid && motion.active()) motion.stop(Result::SensorFault);

            if (std::isfinite(speedChangeDistance) && motion.active() && valid) {
                const Vec current = position(estimate.pose);
                const Vec target = position(motion.request().target);
                const Vec segment = current - speedChangePreviousPosition;
                const double length = segment.dot(segment);
                const double t =
                    length > 1e-9 ? std::clamp((target - speedChangePreviousPosition).dot(segment) / length, 0.0, 1.0)
                                  : 0;
                if ((target - current).norm() <= speedChangeDistance ||
                    (target - (speedChangePreviousPosition + segment * t)).norm() <= speedChangeDistance) {
                    maxMoveSpeed = speedChangePower;
                    speedChangeDistance = unspecified;
                }
                speedChangePreviousPosition = current;
            }

            driveOutput =
                motion.update(estimate, now, dt, std::clamp(maxMoveSpeed.load(), 0.0, 600.0),
                              std::clamp(maxTurnSpeed.load(), 0.0, 600.0), std::clamp(maxMotorSpeed.load(), 0.0, 600.0),
                              driveDisabled.load(), std::clamp(minMoveSpeed.load(), 0.0, 600.0));
            drive.voltage(pros::competition::is_disabled() ? Output{} : driveOutput);
            maximumCycleUs = std::max(maximumCycleUs, static_cast<uint32_t>(pros::micros() - started));
        }
        finishTaskCycle(chassisTaskControl);

        nextTick += config.periodMs;
        const uint32_t finished = pros::millis();
        if (due(finished, nextTick)) {
            const uint32_t skipped = (finished - nextTick) / config.periodMs + 1;
            missedCycles += skipped;
            nextTick += skipped * config.periodMs;
        }
        pros::delay(nextTick - finished);
    }
}

void cascadeTask() {
    bool lastHold = true;
    cascade.hold();

    while (true) {
        if (beginTaskCycle(cascadeTaskControl)) {
            const uint32_t now = pros::millis();
            int voltage = cascadeSpeed.load();
            const uint32_t upUntil = autoLiftUntil.load();
            const uint32_t downUntil = autoDownUntil.load();

            if (upUntil && static_cast<int32_t>(upUntil - now) > 0)
                voltage = autoLiftUpSpeed.load();
            else if (downUntil && static_cast<int32_t>(downUntil - now) > 0)
                voltage = -autoLiftDownSpeed.load();
            if (pros::competition::is_disabled()) voltage = 0;

            cascade.moveCascadeVoltage(voltage);
            const bool hold = cascadeHold.load();
            if (hold != lastHold) {
                if (hold)
                    cascade.hold();
                else
                    cascade.coast();
                lastHold = hold;
            }
            finishTaskCycle(cascadeTaskControl);
        }
        pros::delay(10);
    }
}

void autoClampTask() {
    while (true) {
        if (beginTaskCycle(autoClampTaskControl)) {
            if (!pros::competition::is_disabled()) {
                if (shouldClamp.load() && !hasClamped.load()) {
                    const int distance = claw.distance.get_distance();
                    if (distance >= 20 && distance < 90) {
                        claw.piston.set_value(true);
                        hasClamped = true;
                        if (goUpWhenClamped.load()) autoLiftUntil = pros::millis() + 100;
                    }
                } else if (!shouldClamp.load() && hasClamped.load()) {
                    claw.piston.set_value(false);
                    hasClamped = false;
                }
            }
            finishTaskCycle(autoClampTaskControl);
        }
        pros::delay(10);
    }
}

void printTask() {
    unsigned line = 0;
    while (true) {
        if (beginTaskCycle(printTaskControl)) {
            Pose pose;
            Result result;
            uint32_t cycleUs;
            {
                std::lock_guard<pros::Mutex> lock(chassisMutex);
                pose = odometry.estimate().pose;
                result = motion.result;
                cycleUs = maximumCycleUs;
            }
            if (line == 0)
                controller.print(0, 0, "X%5.1f Y%5.1f H%5.1f ", pose.x, pose.y, wrap(pose.heading));
            else if (line == 1)
                controller.print(1, 0, "%s                  ", resultName(result));
            else
                controller.print(2, 0, "tick %luus            ", static_cast<unsigned long>(cycleUs));
            line = (line + 1) % 3;
            finishTaskCycle(printTaskControl);
        }
        pros::delay(100);
    }
}
