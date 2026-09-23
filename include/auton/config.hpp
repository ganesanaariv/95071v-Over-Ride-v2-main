#pragma once

#include "types.hpp"

// Power is 0–600 (600 = 12000 mV), not measured wheel RPM.
struct Gains {
    double p;
    double d;
    double derivativeFilterSeconds = 0.025;
};

struct Arrival {
    double positionIn = 0.75;
    double headingDeg = 2.0;
    double linearSpeedInPerSec = 1.0;
    double angularSpeedDegPerSec = 5.0;
    uint32_t dwellMs = 120;
};

struct MotionConfig {
    // Translation: P in power/in, D in power/(in/s). Angular gains use degrees.
    Gains translation{35, 0.73};
    Gains turn{7, 0.20};
    Gains drivingHeading{6, 0.15};
    Arrival arrival{};

    double accelerationPowerPerSec = 1500;
    double turnPowerPerSec = 2500;
    double terminalDistanceIn = 6;
    double terminalPower = 160;
    double crossTrackExitIn = 1.5;

    double poseLeadRatio = 0.4;
    double curvePower = 220;
    double progressLeadSeconds = 0.035;
    unsigned maximumReapproaches = 3;
};

struct OdometryConfig {
    double inchesPerTick = 2.75 * pi * (36.0 / 48.0) / 300.0;
    double leftScale = 24.0 / 23.7;
    double rightScale = leftScale;
    double imuScale = 1;
    double velocityFilterSeconds = 0.028;

    // Reject implausible sensor jumps; these do not limit driving speed.
    double maximumWheelSpeedInPerSec = 120;
    double maximumTurnSpeedDegPerSec = 1500;
};

struct SensorMount {
    // Robot frame: origin at drivetrain center, +X right, +Y forward.
    double xRightIn;
    double yForwardIn;
    double headingDeg; // Clockwise: forward 0, right 90, back 180, left 270.
    double biasIn = 0;
    double scale = 1;
};

struct WallResetConfig {
    double maximumCorrectionIn = 15;
    double maximumAngleFromNormalDeg = 30;
    int minimumConfidence = 40; // Sensor confidence 0–63, checked beyond 200 mm.

    // Converted from v1 without changing geometry; verify the direction labels.
    std::array<SensorMount, 4> mounts{{
        // X right, Y forward, beam heading
        {6.12, -0.09, 0},   // Front port
        {6.12, 1.73, 180},  // Back port
        {-5.21, 1.36, 270}, // Right port
        {5.21, 1.38, 90},   // Left port
    }};
};

constexpr double FIELD_GRID_SIZE = 144.0; // Inches, nominal grid.
constexpr double WALL_TO_WALL_SIZE = 140.42;
constexpr double WALL_INSET = (FIELD_GRID_SIZE - WALL_TO_WALL_SIZE) / 2;

struct FieldBounds {
    double left = WALL_INSET;
    double right = FIELD_GRID_SIZE - WALL_INSET;
    double bottom = WALL_INSET;
    double top = FIELD_GRID_SIZE - WALL_INSET;
};

struct Config {
    MotionConfig motion{};
    OdometryConfig odometry{};
    WallResetConfig wall{};
    uint32_t periodMs = 10;
    uint32_t staleMs = 50;
};
const Config config{};

bool validArrival(Arrival arrival) {
    return std::isfinite(arrival.positionIn) && arrival.positionIn > 0 && std::isfinite(arrival.headingDeg) &&
           arrival.headingDeg > 0 && std::isfinite(arrival.linearSpeedInPerSec) && arrival.linearSpeedInPerSec > 0 &&
           std::isfinite(arrival.angularSpeedDegPerSec) && arrival.angularSpeedDegPerSec > 0 && arrival.dwellMs > 0;
}
