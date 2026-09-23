#pragma once

#include "auton/localization.hpp"
#include "auton/motion.hpp"
#include <cassert>
#include <iostream>

void near(double actual, double expected, double tolerance = 1e-6) {
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
        std::cerr << "Expected " << expected << ", got " << actual << '\n';
        std::abort();
    }
}

void testOdometry() {
    Odometry odometry;
    odometry.reset({12, 24, 0});
    assert(odometry.update({0, 0, 80, true}, 100));
    const double ticks = 1 / config.odometry.inchesPerTick;
    assert(odometry.update({ticks, ticks, 80, true}, 110));
    near(odometry.estimate().pose.y, 25);
    near(odometry.estimate().pose.x, 12);

    odometry.reset({50, 60, 90});
    assert(odometry.update({ticks, ticks, 80, true}, 120));
    near(odometry.estimate().velocity.norm(), 0);
    assert(odometry.update({2 * ticks, 2 * ticks, 80, true}, 130));
    near(odometry.estimate().pose.x, 51);
    near(odometry.estimate().pose.y, 60);

    assert(!odometry.update({0, 0, 0, false}, 140));
    assert(odometry.update({10000, 10000, 180, true}, 150));
    near(odometry.estimate().pose.x, 51);
    near(odometry.estimate().pose.heading, 90);
    assert(!odometry.update({10000, 10000, 180, true}, 220));
    near(odometry.estimate().velocity.norm(), 0);

    Odometry arc;
    arc.update({0, 0, 0, true}, 0);
    for (int i = 1; i <= 100; ++i) {
        const double travel = 10 * pi / 2 * i / 100;
        arc.update({travel / config.odometry.inchesPerTick, travel / config.odometry.inchesPerTick, i * 0.9, true},
                   i * 10);
    }
    near(arc.estimate().pose.x, 10);
    near(arc.estimate().pose.y, 10);
    near(arc.estimate().pose.heading, 90);
}

WallReading expectedReading(Pose pose, WallObservation observation, const Config& cfg = config) {
    const auto& mount = cfg.wall.mounts[static_cast<unsigned>(observation.sensor)];
    const Vec f = forward(pose.heading);
    const Vec offset = f * mount.yForwardIn + Vec{f.y, -f.x} * mount.xRightIn;
    const Vec beam = forward(pose.heading + mount.headingDeg);
    const FieldBounds bounds;
    double distance = 0;
    switch (observation.wall) {
    case FieldWall::Left:
        distance = (bounds.left - pose.x - offset.x) / beam.x;
        break;
    case FieldWall::Right:
        distance = (bounds.right - pose.x - offset.x) / beam.x;
        break;
    case FieldWall::Bottom:
        distance = (bounds.bottom - pose.y - offset.y) / beam.y;
        break;
    case FieldWall::Top:
        distance = (bounds.top - pose.y - offset.y) / beam.y;
        break;
    }
    return {distance * 25.4, 63};
}

void testWallGeometry() {
    const Pose truth{120, 24, 180};
    const WallObservation right{WallSensor::Right, FieldWall::Right};
    const WallObservation bottom{WallSensor::Back, FieldWall::Bottom};
    const auto x = solveWall({115, 24, 180}, right, expectedReading(truth, right), {});
    const auto y = solveWall({120, 29, 180}, bottom, expectedReading(truth, bottom), {});
    assert(x.valid && y.valid);
    near(x.coordinate, 120);
    near(y.coordinate, 24);
    assert(!solveWall(truth, right, {3000, 63}, {}).valid);
    assert(!solveWall(truth, right, {300, 0}, {}).valid);
    assert(!solveWall(truth, right, {300, 2147483647}, {}).valid);
    assert(!solveWall({120, 24, 0}, right, {300, 63}, {}).valid);
    assert(!solveWall({120, -24, 180}, right, {300, 63}, {}).valid);

    const Pose tilted{120, 24, 165};
    const auto rotated = solveWall({115, 24, 165}, right, expectedReading(tilted, right), {});
    assert(rotated.valid);
    near(rotated.coordinate, 120);

    Config shortRange = config;
    shortRange.wall.mounts[0] = {0, 0, 180};
    assert(solveWall({72, 5, 0}, {WallSensor::Front, FieldWall::Bottom}, {81.534, 0}, {}, shortRange).valid);

    WallReset reset;
    WallRequest request;
    request.count = 2;
    request.observations = {right, bottom};
    reset.start(request, 0);
    Estimate estimate;
    estimate.valid = true;
    estimate.pose = {115, 29, 180};
    estimate.sampledAt = 10;
    assert(reset.wantsSample(estimate, 10));
    reset.sample(estimate, {expectedReading(truth, right), expectedReading(truth, bottom)}, {}, 10);
    assert(reset.result == Result::Applied);
    near(reset.corrected.x, 120);
    near(reset.corrected.y, 24);

    reset.start(request, 0);
    reset.sample(estimate, {expectedReading(truth, right), {4000, 63}}, {}, 10);
    assert(reset.result == Result::Rejected);
    near(estimate.pose.x, 115);
    near(estimate.pose.y, 29);

    request.timeout = 150;
    reset.start(request, 0);
    estimate.velocity = {3, 0};
    estimate.sampledAt = 149;
    assert(!reset.wantsSample(estimate, 149));
    assert(reset.result == Result::Running);
    estimate.velocity = {};
    estimate.sampledAt = 150;
    reset.sample(estimate, {expectedReading(truth, right), expectedReading(truth, bottom)}, {}, 150);
    assert(reset.result == Result::TimedOut);

    request.timeout = 0;
    reset.start(request, 0);
    estimate.sampledAt = 6000;
    assert(reset.wantsSample(estimate, 6000));
    reset.sample(estimate, {expectedReading(truth, right), expectedReading(truth, bottom)}, {}, 6000);
    assert(reset.result == Result::Applied);

    request.maximumCorrectionIn = 2;
    reset.start(request, 6000);
    reset.sample(estimate, {expectedReading(truth, right), expectedReading(truth, bottom)}, {}, 6000);
    assert(reset.result == Result::Rejected);

    Config mounts = config;
    mounts.wall.mounts[0] = {3, 5, 0};
    const double range = (FieldBounds{}.top - 72 - 5) * 25.4;
    const auto north = solveWall({72, 70, 0}, {WallSensor::Front, FieldWall::Top}, {range, 63}, {}, mounts);
    const auto east = solveWall({70, 72, 90}, {WallSensor::Front, FieldWall::Right}, {range, 63}, {}, mounts);
    assert(north.valid && east.valid);
    near(north.coordinate, 72);
    near(east.coordinate, 72);
    mounts.wall.mounts[0].headingDeg = 90;
    const auto rightFacing = solveWall({70, 72, 0}, {WallSensor::Front, FieldWall::Right},
                                       {(FieldBounds{}.right - 72 - 3) * 25.4, 63}, {}, mounts);
    assert(rightFacing.valid);
    near(rightFacing.coordinate, 72);
}

void testSettlingAndOutput() {
    SettlingWindow window;
    assert(!window.update(true, 0, 100));
    assert(!window.update(true, 50, 100));
    assert(window.update(true, 100, 100));
    assert(!window.update(false, 110, 100));
    assert(!window.update(true, 120, 100));
    assert(!window.update(true, 220, 100));
    assert(!window.update(true, 250, 100));

    assert(elapsed(10, UINT32_MAX - 9, 20));
    assert(due(10, UINT32_MAX - 9));
    near(slewPower(-100, 100, 1000, .01), 0);
    near(slewPower(10, 100, 1000, .01), 10);
    const Output output = mix(600, 400, 600);
    near(output.left, 600);
    near(output.right, -200);
}

void testMeasuredTime() {
    PD derivative({0, 1, 0});
    near(derivative.update(10, .01), 0);
    near(derivative.update(9.9, .01), -10);
    near(derivative.update(9.7, .02), -10);
    derivative.reset();
    near(derivative.update(90, .01, true), 0);
    derivative.update(179, .01, true);
    near(derivative.update(-179, .01, true), 200);

    Odometry odometry;
    odometry.update({0, 0, 0, true}, 0);
    for (uint32_t now : {7u, 20u, 29u, 43u, 50u, 65u, 80u, 100u}) {
        const double ticks = (now * .02) / config.odometry.inchesPerTick;
        assert(odometry.update({ticks, ticks, 0, true}, now));
    }
    near(odometry.estimate().pose.y, 2);
    assert(odometry.estimate().velocity.y > 18 && odometry.estimate().velocity.y < 20);
}

void testMotionDeadlines() {
    Estimate estimate;
    estimate.valid = true;
    MotionRequest request;
    request.target = {0, 50, 0};
    request.options.timeout = 100;
    Motion motion;
    motion.start(request, estimate, 0);
    for (uint32_t now = 10; now <= 100; now += 10) {
        estimate.sampledAt = now;
        near(motion.update(estimate, now, .01, 600, 600, 600, true).left, 0);
    }
    assert(motion.result == Result::TimedOut);

    request.options.timeout = 0;
    motion.start(request, estimate, 100);
    estimate.valid = false;
    near(motion.update(estimate, 110, .01, 600, 600, 600, false).left, 0);
    assert(motion.result == Result::SensorFault);
    estimate.valid = true;
    motion.start(request, estimate, 100);
    for (uint32_t now = 110; now < 1500; now += 10) {
        estimate.sampledAt = now;
        motion.update(estimate, now, .01, 600, 600, 600, false);
    }
    assert(motion.result == Result::Running);
    estimate.sampledAt = 10000;
    assert(motion.update(estimate, 10000, .01, 600, 600, 600, false).left > 0);
    assert(motion.result == Result::Running);

    request.target = {0, 2, 0};
    request.options = chainOptions(100, 3);
    estimate.sampledAt = 0;
    motion.start(request, estimate, 0);
    motion.update(estimate, 0, .01, 600, 600, 600, false);
    assert(motion.result == Result::HandedOff);
    estimate.sampledAt = 10000;
    assert(motion.update(estimate, 10000, .01, 600, 600, 600, false).left > 0);
    assert(motion.result == Result::HandedOff);
}

void testMovePowerLimits() {
    for (double direction : {1.0, -1.0}) {
        Motion motion;
        Estimate estimate;
        estimate.valid = true;
        MotionRequest request;
        request.kind = MotionKind::Distance;
        request.distance = direction * 2;
        motion.start(request, estimate, 0);
        Output output;
        for (uint32_t now = 10; now <= 100; now += 10) {
            estimate.sampledAt = now;
            output = motion.update(estimate, now, .01, 100, 600, 600, false, 80);
            assert(std::fabs(output.left) <= 100);
        }
        near(output.left, direction * 80);
        estimate.sampledAt = 110;
        output = motion.update(estimate, 110, .01, 50, 600, 600, false, 80);
        near(output.left, direction * 50);
        estimate.sampledAt = 120;
        near(motion.update(estimate, 120, .01, 100, 600, 600, true, 80).left, 0);
        estimate.travel = direction * 2;
        for (uint32_t now = 130; now <= 260; now += 10) {
            estimate.sampledAt = now;
            near(motion.update(estimate, now, .01, 100, 600, 600, false, 80).left, 0);
        }
        assert(motion.result == Result::Settled);
    }
}

struct MotionSimulation {
    Estimate estimate{};
    Motion motion;
    double leftSpeed = 0, rightSpeed = 0;
    uint32_t now = 0;
    MotionSimulation(Pose pose = {}) {
        estimate.pose = pose;
        estimate.valid = true;
    }
    void start(MotionRequest request) {
        motion.start(request, estimate, now);
    }
    void step(uint32_t milliseconds = 10) {
        const double dt = milliseconds / 1000.0;
        now += milliseconds;
        estimate.sampledAt = now;
        const Output output = motion.update(estimate, now, dt, 400, 350, 600, false);
        assert(std::fabs(output.left) <= 600 && std::fabs(output.right) <= 600);
        leftSpeed += (output.left * 69.1 / 600 - leftSpeed) * dt / (.0566666667 + dt);
        rightSpeed += (output.right * 69.1 / 600 - rightSpeed) * dt / (.0566666667 + dt);
        const double linear = (leftSpeed + rightSpeed) / 2;
        const double angular = (leftSpeed - rightSpeed) / 12 / radians;
        const Vec velocity = forward(estimate.pose.heading + angular * dt / 2) * linear;
        estimate.pose.x += velocity.x * dt;
        estimate.pose.y += velocity.y * dt;
        estimate.pose.heading += angular * dt;
        estimate.travel += linear * dt;
        estimate.velocity = velocity;
        estimate.angularRate = angular;
    }
    void settle(const char* name, bool jitter = false) {
        for (int i = 0; i < 600 && motion.active(); ++i)
            step(jitter ? (i % 2 ? 14 : 6) : 10);
        if (motion.result != Result::Settled) {
            std::cerr << name << " failed: " << resultName(motion.result) << " at " << estimate.pose.x << ','
                      << estimate.pose.y << ',' << estimate.pose.heading << '\n';
            std::abort();
        }
        assert(motion.arrived(estimate, motion.request().options.arrival));
    }
};

void testMotionSimulation() {
    for (Direction direction : {Direction::Forward, Direction::Reverse}) {
        MotionSimulation sim({0, 0, direction == Direction::Forward ? 0.0 : 180.0});
        MotionRequest request;
        request.target = {12, 36, 0};
        request.options.direction = direction;
        sim.start(request);
        sim.settle("point");
        assert((position(sim.estimate.pose) - position(request.target)).norm() <= .75);
    }
    for (double distance : {24.0, -24.0, 1.0, -1.0}) {
        MotionSimulation sim;
        MotionRequest request;
        request.kind = MotionKind::Distance;
        request.distance = distance;
        sim.start(request);
        sim.settle("distance");
    }
    for (double target : {90.0, -90.0, 180.0, 350.0}) {
        MotionSimulation sim;
        MotionRequest request;
        request.kind = MotionKind::Turn;
        request.target.heading = target;
        sim.start(request);
        sim.settle("turn");
    }
    for (Direction direction : {Direction::Forward, Direction::Reverse}) {
        MotionSimulation sim({0, 0, direction == Direction::Forward ? 0.0 : 180.0});
        MotionRequest request;
        request.kind = MotionKind::Pose;
        request.target = {12, 36, direction == Direction::Forward ? 30.0 : 210.0};
        request.options.direction = direction;
        sim.start(request);
        sim.settle("pose");
    }

    MotionSimulation jitter;
    MotionRequest jitterRequest;
    jitterRequest.kind = MotionKind::Pose;
    jitterRequest.target = {12, 36, 30};
    jitter.start(jitterRequest);
    jitter.settle("variable timestep pose", true);

    MotionSimulation directedTurn;
    MotionRequest turnRequest;
    turnRequest.kind = MotionKind::Turn;
    turnRequest.target.heading = -90;
    turnRequest.options.turn = TurnDirection::Clockwise;
    directedTurn.start(turnRequest);
    directedTurn.settle("clockwise long turn");
    near(directedTurn.estimate.pose.heading, 270, 2);

    MotionSimulation overshoot;
    MotionRequest request;
    request.target = {0, 4, 0};
    overshoot.start(request);
    overshoot.step();
    overshoot.estimate.pose.y = 5;
    overshoot.settle("overshoot");

    MotionSimulation lateral;
    lateral.start(request);
    lateral.step();
    lateral.estimate.pose = {3, 4, 0};
    lateral.settle("cross track");
}

int main() {
    testOdometry();
    testWallGeometry();
    testSettlingAndOutput();
    testMeasuredTime();
    testMotionDeadlines();
    testMovePowerLimits();
    testMotionSimulation();
    std::cout << "Autonomous core tests passed\n";
}
