#pragma once

#include "config.hpp"

struct WallRequest {
    std::array<WallObservation, 2> observations{};
    unsigned count = 1;
    double maximumCorrectionIn = config.wall.maximumCorrectionIn;
};

class Odometry {
    const Config& cfg;
    SensorSample previous{};
    uint32_t previousTime = 0;
    bool baseline = false;
    double headingOffset = 0;
    Estimate state{};

public:
    explicit Odometry(const Config& settings = config) : cfg(settings) {}
    const Estimate& estimate() const {
        return state;
    }

    void rebase() {
        baseline = false;
        state.valid = false;
        state.velocity = {};
        state.angularRate = 0;
    }

    void reset(Pose pose) {
        if (baseline)
            headingOffset = pose.heading - previous.rotation * cfg.odometry.imuScale;
        else
            headingOffset = pose.heading;
        state.pose = pose;
        state.velocity = {};
        state.angularRate = 0;
    }

    bool update(SensorSample sample, uint32_t now) {
        state.valid = false;
        if (!sample.valid || !std::isfinite(sample.left) || !std::isfinite(sample.right) ||
            !std::isfinite(sample.rotation)) {
            baseline = false;
            state.velocity = {};
            state.angularRate = 0;
            return false;
        }
        if (!baseline) {
            headingOffset = state.pose.heading - sample.rotation * cfg.odometry.imuScale;
            previous = sample;
            previousTime = now;
            baseline = true;
            state.sampledAt = now;
            state.valid = true;
            return true;
        }
        const uint32_t dtMs = now - previousTime;
        if (!dtMs) return false;
        const double dt = dtMs / 1000.0;
        const double left = (sample.left - previous.left) * cfg.odometry.inchesPerTick * cfg.odometry.leftScale;
        const double right = (sample.right - previous.right) * cfg.odometry.inchesPerTick * cfg.odometry.rightScale;
        const double dh = (sample.rotation - previous.rotation) * cfg.odometry.imuScale;
        previous = sample;
        previousTime = now;
        if (dtMs > cfg.staleMs || std::fabs(left) / dt > cfg.odometry.maximumWheelSpeedInPerSec ||
            std::fabs(right) / dt > cfg.odometry.maximumWheelSpeedInPerSec ||
            std::fabs(dh) / dt > cfg.odometry.maximumTurnSpeedDegPerSec) {
            headingOffset = state.pose.heading - sample.rotation * cfg.odometry.imuScale;
            state.velocity = {};
            state.angularRate = 0;
            return false;
        }
        const double ds = (left + right) / 2;
        const double half = dh * radians / 2;
        const double sinc = std::fabs(half) < 1e-6 ? 1 - half * half / 6 : std::sin(half) / half;
        const Vec delta = forward(state.pose.heading + dh / 2) * (ds * sinc);
        state.pose.x += delta.x;
        state.pose.y += delta.y;
        state.pose.heading = sample.rotation * cfg.odometry.imuScale + headingOffset;
        state.travel += ds;
        const double alpha = dt / (cfg.odometry.velocityFilterSeconds + dt);
        state.velocity = state.velocity + (delta * (1 / dt) - state.velocity) * alpha;
        state.angularRate += alpha * (dh / dt - state.angularRate);
        state.sampledAt = now;
        state.valid = true;
        return true;
    }
};

struct WallSolution {
    bool valid = false;
    double coordinate = 0;
};
bool vertical(FieldWall wall) {
    return wall == FieldWall::Left || wall == FieldWall::Right;
}
WallSolution solveWall(Pose pose, WallObservation observation, WallReading reading, FieldBounds bounds,
                       const Config& cfg = config) {
    if (!finite(pose) || !std::isfinite(reading.millimeters) || reading.millimeters < 20 || reading.millimeters > 2000)
        return {};
    if (reading.millimeters > 200 && (reading.confidence < cfg.wall.minimumConfidence || reading.confidence > 63))
        return {};
    if (static_cast<unsigned>(observation.sensor) >= cfg.wall.mounts.size() ||
        static_cast<unsigned>(observation.wall) > 3)
        return {};
    const auto& mount = cfg.wall.mounts[static_cast<unsigned>(observation.sensor)];
    const Vec f = forward(pose.heading);
    const Vec right{f.y, -f.x};
    const Vec offset = right * mount.xRightIn + f * mount.yForwardIn;
    const Vec beam = forward(pose.heading + mount.headingDeg);
    const double range = reading.millimeters / 25.4 * mount.scale + mount.biasIn;
    const bool xAxis = vertical(observation.wall);
    double wall = 0, normal = 0;
    switch (observation.wall) {
    case FieldWall::Left:
        wall = bounds.left;
        normal = -beam.x;
        break;
    case FieldWall::Right:
        wall = bounds.right;
        normal = beam.x;
        break;
    case FieldWall::Bottom:
        wall = bounds.bottom;
        normal = -beam.y;
        break;
    case FieldWall::Top:
        wall = bounds.top;
        normal = beam.y;
        break;
    }
    if (normal < std::cos(cfg.wall.maximumAngleFromNormalDeg * radians) || range <= 0) return {};
    const double coordinate = wall - (xAxis ? offset.x + range * beam.x : offset.y + range * beam.y);
    const Vec origin{xAxis ? coordinate + offset.x : pose.x + offset.x,
                     xAxis ? pose.y + offset.y : coordinate + offset.y};
    const Vec hit = origin + beam * range;
    constexpr double epsilon = 0.001;
    if (origin.x < bounds.left || origin.x > bounds.right || origin.y < bounds.bottom || origin.y > bounds.top)
        return {};
    if (xAxis ? (hit.y <= bounds.bottom + epsilon || hit.y >= bounds.top - epsilon)
              : (hit.x <= bounds.left + epsilon || hit.x >= bounds.right - epsilon))
        return {};
    return {true, coordinate};
}
