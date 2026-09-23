#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

constexpr double pi = 3.14159265358979323846;
constexpr double radians = pi / 180.0;
constexpr double unspecified = std::numeric_limits<double>::quiet_NaN();
double wrap(double degrees) {
    return std::remainder(degrees, 360.0);
}
double bearing(double dx, double dy) {
    return std::atan2(dx, dy) / radians;
}
double limit(double value, double cap) {
    return std::clamp(value, -cap, cap);
}
int sign(double value) {
    return (value > 0) - (value < 0);
}
bool elapsed(uint32_t now, uint32_t start, uint32_t duration) {
    return now - start >= duration;
}
bool due(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

struct Pose {
    double x = 0, y = 0, heading = 0;
};
struct Vec {
    double x = 0, y = 0;
    Vec operator+(Vec b) const {
        return {x + b.x, y + b.y};
    }
    Vec operator-(Vec b) const {
        return {x - b.x, y - b.y};
    }
    Vec operator*(double k) const {
        return {x * k, y * k};
    }
    double dot(Vec b) const {
        return x * b.x + y * b.y;
    }
    double norm() const {
        return std::hypot(x, y);
    }
};
Vec forward(double heading) {
    return {std::sin(heading * radians), std::cos(heading * radians)};
}
Vec position(Pose pose) {
    return {pose.x, pose.y};
}
bool finite(Pose pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.heading);
}

enum class Direction { Default, Auto, Forward, Reverse };
enum class TurnDirection { Shortest, Clockwise, Counterclockwise };
enum class MotionKind { Point, Pose, Turn, Distance };
enum class MotionMode { Stop, Chain };
enum class Result {
    Idle,
    Running,
    Settled,
    HandedOff,
    Applied,
    Rejected,
    TimedOut,
    Stalled,
    SensorFault,
    Invalid,
};
enum class WallSensor : uint8_t { Front, Back, Right, Left };
enum class FieldWall : uint8_t { Left, Right, Bottom, Top };

const char* resultName(Result result) {
    switch (result) {
    case Result::Idle:
        return "idle";
    case Result::Running:
        return "running";
    case Result::Settled:
        return "settled";
    case Result::HandedOff:
        return "handoff";
    case Result::Applied:
        return "applied";
    case Result::Rejected:
        return "reset rejected";
    case Result::TimedOut:
        return "timeout";
    case Result::Stalled:
        return "stalled";
    case Result::SensorFault:
        return "sensor fault";
    case Result::Invalid:
        return "invalid request";
    default:
        return "unexpected failure";
    }
}

struct SensorSample {
    double left = 0, right = 0, rotation = 0;
    bool valid = false;
};
struct Estimate {
    Pose pose{};
    Vec velocity{};
    double angularRate = 0, travel = 0;
    uint32_t sampledAt = 0;
    bool valid = false;
};
struct Output {
    double left = 0, right = 0;
};
struct WallReading {
    double millimeters = 0;
    int confidence = 0;
};
struct WallObservation {
    WallSensor sensor = WallSensor::Front;
    FieldWall wall = FieldWall::Bottom;
};
