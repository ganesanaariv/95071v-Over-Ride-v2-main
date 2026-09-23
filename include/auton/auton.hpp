#pragma once

#include "tasks.hpp"

Pose getRobotPose() {
    std::lock_guard<pros::Mutex> lock(chassisMutex);
    return odometry.estimate().pose;
}

void setMoveSpeed(double maximum, double minimum = 0) {
    maxMoveSpeed = std::isfinite(maximum) ? std::clamp(maximum, 0.0, 600.0) : 0;
    minMoveSpeed = std::isfinite(minimum) ? std::clamp(minimum, 0.0, maxMoveSpeed.load()) : 0;
}

void setTurnSpeed(double power) {
    maxTurnSpeed = std::isfinite(power) ? std::clamp(power, 0.0, 600.0) : 0;
}

void setDriveMode(Direction direction) {
    driveMode = direction;
}

void pauseDrive() {
    driveDisabled = true;
}

void resumeDrive() {
    driveDisabled = false;
}

void setPos(double x, double y, double heading) {
    std::lock_guard<pros::Mutex> lock(chassisMutex);
    if (!finite({x, y, heading})) return;
    odometry.rebase();
    odometry.reset({x, y, heading});
    motion.stop();
    driveOutput = {};
    speedChangeDistance = unspecified;
    desiredHeading = heading;
}

void setPos(double x, double y) {
    setPos(x, y, getRobotPose().heading);
}

void startMotion(MotionRequest request) {
    std::lock_guard<pros::Mutex> lock(chassisMutex);
    const auto& estimate = odometry.estimate();
    if (request.options.direction == Direction::Default) request.options.direction = driveMode.load();
    if (request.kind == MotionKind::Distance) {
        if (!std::isfinite(request.target.heading))
            request.target.heading = std::isfinite(desiredHeading) ? desiredHeading : estimate.pose.heading;
        const Vec target = position(estimate.pose) + forward(request.target.heading) * request.distance;
        request.target.x = target.x;
        request.target.y = target.y;
    }
    motion.start(request, estimate, pros::millis());
    if (request.kind == MotionKind::Turn || request.kind == MotionKind::Pose || request.kind == MotionKind::Distance)
        desiredHeading = request.target.heading;
    speedChangeDistance = unspecified;
    driveOutput = {};
}

void setTargetPos(double x, double y, MotionOptions options = {}) {
    MotionRequest request;
    request.target = {x, y, 0};
    request.options = options;
    startMotion(request);
}

void setTargetPose(double x, double y, double heading, MotionOptions options = {}) {
    MotionRequest request;
    request.kind = MotionKind::Pose;
    request.target = {x, y, heading};
    request.options = options;
    startMotion(request);
}

void setTargetHeading(double heading, MotionOptions options = {}) {
    MotionRequest request;
    request.kind = MotionKind::Turn;
    request.target = getRobotPose();
    request.target.heading = heading;
    request.options = options;
    startMotion(request);
}

void facePoint(double x, double y, Direction direction = Direction::Forward, MotionOptions options = {}) {
    const Pose pose = getRobotPose();
    double heading = bearing(x - pose.x, y - pose.y);
    if (direction == Direction::Reverse ||
        (direction == Direction::Auto && std::fabs(wrap(heading - pose.heading)) > 90))
        heading += 180;
    setTargetHeading(heading, options);
}

void driveDistance(double distance, double heading = unspecified, MotionOptions options = {}) {
    MotionRequest request;
    request.kind = MotionKind::Distance;
    request.distance = distance;
    request.target.heading = heading;
    request.options = options;
    startMotion(request);
}

void untilDelay(uint32_t milliseconds, bool halt = true) {
    if (halt) pauseDrive();
    pros::delay(milliseconds);
}

bool untilTargetPos(double tolerance, uint32_t timeout = 0, uint32_t extraTime = 0) {
    if (!std::isfinite(tolerance) || tolerance < 0) return false;
    const uint32_t started = pros::millis();
    bool reached = false;
    Vec previous = position(getRobotPose());
    while (chassisTaskControl.enabled.load()) {
        {
            std::lock_guard<pros::Mutex> lock(chassisMutex);
            const auto& estimate = odometry.estimate();
            const Vec current = position(estimate.pose);
            const Vec delta = position(motion.request().target) - current;
            const double distance = delta.norm();
            const double closing = distance > 1e-6 ? std::max(0.0, delta.dot(estimate.velocity) / distance) : 0;
            if (motion.request().kind == MotionKind::Distance) {
                reached = std::fabs(motion.distanceRemaining(estimate)) <=
                          tolerance + closing * config.motion.progressLeadSeconds;
            } else {
                const Vec segment = current - previous;
                const double length = segment.dot(segment);
                const double t =
                    length > 1e-9
                        ? std::clamp((position(motion.request().target) - previous).dot(segment) / length, 0.0, 1.0)
                        : 0;
                reached = distance <= tolerance + closing * config.motion.progressLeadSeconds ||
                          (position(motion.request().target) - (previous + segment * t)).norm() <= tolerance;
            }
            reached = reached && estimate.valid && !elapsed(pros::millis(), estimate.sampledAt, config.staleMs + 1);
            previous = current;
            if (reached || (!motion.active() && motion.result != Result::Settled)) break;
        }
        if (timeout && elapsed(pros::millis(), started, timeout)) break;
        pros::delay(2);
    }
    if (extraTime) pros::delay(extraTime);
    return reached;
}

bool untilTargetH(double tolerance, uint32_t timeout = 0, uint32_t extraTime = 0) {
    if (!std::isfinite(tolerance) || tolerance < 0) return false;
    const uint32_t started = pros::millis();
    bool reached = false;
    while (chassisTaskControl.enabled.load()) {
        {
            std::lock_guard<pros::Mutex> lock(chassisMutex);
            const auto& estimate = odometry.estimate();
            const auto& request = motion.request();
            const Vec delta = position(request.target) - position(estimate.pose);
            const double heading = request.kind == MotionKind::Turn || request.kind == MotionKind::Pose ||
                                           request.kind == MotionKind::Distance
                                       ? request.target.heading
                                       : bearing(delta.x, delta.y) + (motion.capturedDirection() < 0 ? 180 : 0);
            reached = estimate.valid && !elapsed(pros::millis(), estimate.sampledAt, config.staleMs + 1) &&
                      std::fabs(wrap(heading - estimate.pose.heading)) <= tolerance;
            if (reached || (!motion.active() && motion.result != Result::Settled)) break;
        }
        if (timeout && elapsed(pros::millis(), started, timeout)) break;
        pros::delay(2);
    }
    if (extraTime) pros::delay(extraTime);
    return reached;
}

bool untilSettled(Arrival arrival, uint32_t timeout) {
    if (!validArrival(arrival)) return false;
    {
        std::lock_guard<pros::Mutex> lock(chassisMutex);
        if (motion.request().options.mode == MotionMode::Chain) return false;
        if (motion.active()) motion.setArrival(arrival);
    }
    SettlingWindow window;
    const uint32_t started = pros::millis();
    while (chassisTaskControl.enabled.load() && !driveDisabled.load()) {
        {
            std::lock_guard<pros::Mutex> lock(chassisMutex);
            const auto& estimate = odometry.estimate();
            const bool fresh = !elapsed(pros::millis(), estimate.sampledAt, config.staleMs + 1);
            if (window.update(fresh && motion.arrived(estimate, arrival), estimate.sampledAt, arrival.dwellMs,
                              config.staleMs))
                return true;
            if (!motion.active() && motion.result != Result::Settled) return false;
        }
        if (timeout && elapsed(pros::millis(), started, timeout)) return false;
        pros::delay(10);
    }
    return false;
}

bool untilTargetSettled(double tolerance = config.motion.arrival.positionIn, uint32_t timeout = 0) {
    Arrival arrival = config.motion.arrival;
    arrival.positionIn = tolerance;
    return untilSettled(arrival, timeout);
}

bool untilHeadingSettled(double tolerance = config.motion.arrival.headingDeg, uint32_t timeout = 0) {
    {
        std::lock_guard<pros::Mutex> lock(chassisMutex);
        if (motion.request().kind != MotionKind::Turn) return false;
    }
    Arrival arrival = config.motion.arrival;
    arrival.headingDeg = tolerance;
    return untilSettled(arrival, timeout);
}

bool untilClamped(uint32_t timeout = 0, uint32_t extraTime = 0) {
    const uint32_t started = pros::millis();
    while (!hasClamped.load()) {
        if (timeout && elapsed(pros::millis(), started, timeout)) break;
        pros::delay(2);
    }
    if (extraTime) pros::delay(extraTime);
    return hasClamped.load();
}

void setMoveSpeedAtDistance(double distance, double power) {
    std::lock_guard<pros::Mutex> lock(chassisMutex);
    if (!std::isfinite(distance) || distance < 0 || !std::isfinite(power)) return;
    speedChangeDistance = distance;
    speedChangePower = std::clamp(power, 0.0, 600.0);
    speedChangePreviousPosition = position(odometry.estimate().pose);
}

Result resetFromWalls(WallRequest request) {
    std::lock_guard<pros::Mutex> lock(chassisMutex);
    driveDisabled = true;
    motion.stop();
    driveOutput = {};
    speedChangeDistance = unspecified;

    const Estimate& estimate = odometry.estimate();
    if (!estimate.valid || elapsed(pros::millis(), estimate.sampledAt, config.staleMs + 1)) return Result::SensorFault;
    if (request.count < 1 || request.count > 2 || !std::isfinite(request.maximumCorrectionIn) ||
        request.maximumCorrectionIn <= 0 ||
        (request.count == 2 && vertical(request.observations[0].wall) == vertical(request.observations[1].wall)))
        return Result::Invalid;

    Pose corrected = estimate.pose;
    for (unsigned i = 0; i < request.count; ++i) {
        const WallSolution solution = solveWall(estimate.pose, request.observations[i],
                                               drive.wallReading(request.observations[i].sensor), FieldBounds{});
        const bool xAxis = vertical(request.observations[i].wall);
        const double previous = xAxis ? estimate.pose.x : estimate.pose.y;
        if (!solution.valid || std::fabs(solution.coordinate - previous) > request.maximumCorrectionIn)
            return Result::Rejected;
        if (xAxis)
            corrected.x = solution.coordinate;
        else
            corrected.y = solution.coordinate;
    }
    odometry.reset(corrected);
    return Result::Applied;
}

Result resetFromWall(WallSensor sensor, FieldWall wall) {
    WallRequest request;
    request.observations[0] = {sensor, wall};
    return resetFromWalls(request);
}
