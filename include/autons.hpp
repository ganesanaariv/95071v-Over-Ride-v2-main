#pragma once

#include "auton/auton.hpp"

void exampleIdle() {}

void exampleStraight() {
    setPos(72, 24, 0);
    setMoveSpeed(250);
    driveDistance(24);
    if (!untilTargetSettled(0.75, 2500)) return;

    driveDistance(-12);
    untilTargetSettled(0.75, 2000);
}

void examplePoint() {
    setPos(72, 24, 0);
    setDriveMode(Direction::Forward);
    setMoveSpeed(350);

    setTargetPos(96, 48);
    if (!untilTargetPos(12, 2000)) return;
    setMoveSpeed(150);
    if (!untilTargetSettled(0.75, 2000)) return;

    setTargetHeading(90);
    untilHeadingSettled(2, 1500);
}

void examplePose() {
    setPos(72, 24, 0);
    setDriveMode(Direction::Reverse);
    setMoveSpeed(250);
    setTargetPose(60, 12, 90);
    untilTargetSettled(1, 3500);
}

void exampleChain() {
    setPos(72, 24, 0);
    setDriveMode(Direction::Forward);
    setMoveSpeed(300);

    setTargetPos(72, 48, chainOptions(120, 4));
    if (!untilTargetPos(4, 2000)) return;
    setTargetPos(84, 72);
    setMoveSpeedAtDistance(12, 150);
    untilTargetSettled(0.75, 3000);
}

void exampleWallReset() {
    setPos(72, 24, 0);
    if (resetFromWall(WallSensor::Front, FieldWall::Bottom) != Result::Applied) return;

    setMoveSpeed(200);
    setTargetPos(72, 36);
    resumeDrive();
    untilTargetSettled(0.75, 2000);
}

void exampleMechanisms() {
    shouldClamp = true;
    if (!untilClamped(1500)) return;

    autoLiftUntil = pros::millis() + 300;
    untilDelay(400, false);
    autoDownUntil = pros::millis() + 300;
    untilDelay(400, false);

    shouldClamp = false;
    untilDelay(20, false);
}

void rightSideToggle() {
    // Begin near the bottom wall, then use the front distance sensor to
    // correct the starting Y coordinate from its measured range.
    shouldClamp = true;
    setPos(3 * 24, 12, 0);
    resetFromWall(WallSensor::Front, FieldWall::Bottom);
    untilDelay(50, true);
    resumeDrive();

    enableTask(cascadeTaskControl);
    enableTask(autoClampTaskControl);
    setDriveMode(Direction::Auto);
    setMoveSpeed(600, 200);
    cascadeHold = true;

    const double startX = getRobotPose().x;
    setTargetPos(startX, 14);
    untilTargetPos(3, 1200);
    setTargetPos(startX, 0);
    untilTargetPos(3, 400);
    setTargetPos(startX, 14);
    untilTargetPos(3, 1200);
    setTargetPos(startX, 0);
    untilTargetPos(3, 450);
    autoLiftUntil = pros::millis() + 300;
    setMoveSpeed(600, 0);

    untilDelay(100, true);
    resetFromWall(WallSensor::Front, FieldWall::Bottom);
    untilDelay(50, true);
    resumeDrive();

    setMoveSpeed(300);
    setTargetPos(3 * 24, 24);
    untilTargetPos(2, 1000);

    setMoveSpeed(600);
    setTargetPos(4 * 24 + 1, 24);
    setMoveSpeed(0);
    untilTargetH(3, 1000, 100);
    setMoveSpeed(600);
    untilTargetPos(14, 1000);
    setMoveSpeed(150);
    untilTargetPos(3, 400);
    pauseDrive();
    autoDownUntil = pros::millis() + 500;

    pros::delay(100);
    // Square the pose against the right and bottom field walls.
    WallRequest rightBottom;
    rightBottom.observations[0] = {WallSensor::Front, FieldWall::Right};
    rightBottom.observations[1] = {WallSensor::Right, FieldWall::Bottom};
    rightBottom.count = 2;
    resetFromWalls(rightBottom);
    pros::delay(50);
    shouldClamp = false;
    untilDelay(350, true);

    resumeDrive();
    setDriveMode(Direction::Reverse);
    setTargetPos(3 * 24 + 6, 24 - 1);
    untilTargetPos(3, 1200);

    setDriveMode(Direction::Forward);
    setMoveSpeed(0);
    setTargetPos(4 * 24, 2 * 24);
    untilTargetH(1, 800, 100);
    setMoveSpeed(450);
    shouldClamp = true;
    untilTargetPos(14, 1000);
    setMoveSpeed(100);
    untilClamped(2000);
    autoLiftUntil = pros::millis() + 650;
    untilDelay(100, true);

    setTargetPos(4 * 24, 24);
    autoLiftDownSpeed = 6000;
    resumeDrive();
    setMoveSpeed(0);
    untilTargetH(1, 1000);
    setMoveSpeed(300);
    untilTargetPos(10, 1000);
    setMoveSpeed(150);
    untilTargetPos(3, 500);
    pauseDrive();
    autoDownUntil = pros::millis() + 300;

    pros::delay(100);
    resetFromWall(WallSensor::Front, FieldWall::Bottom);
    resetFromWall(WallSensor::Left, FieldWall::Right);
    pros::delay(300);
    untilDelay(200, true);
    shouldClamp = false;
    claw.piston.set_value(false);
    autoDownUntil = pros::millis() + 200;
    untilDelay(200, true);

    setDriveMode(Direction::Reverse);
    resumeDrive();
    const double currentX = getRobotPose().x;
    setTargetPos(currentX, 2 * 24);
    setMoveSpeed(300);
    untilTargetPos(3, 1000);

    shouldClamp = true;
    setDriveMode(Direction::Forward);
    setTargetPos(5 * 24, 24);
    setMoveSpeed(0);
    untilTargetH(3, 1000);
    setMoveSpeed(300);
    untilTargetPos(16, 1000);
    setMoveSpeed(150);
    untilClamped(1200);
    untilDelay(100, true);
    resumeDrive();
    autoLiftUntil = pros::millis() + 750;

    setTargetPos(4 * 24 - 1, 24 + 1.25);
    autoLiftDownSpeed = 6000;
    setMoveSpeed(0);
    untilTargetH(3, 1000);
    setMoveSpeed(300);
    untilTargetPos(8, 600);
    setMoveSpeed(150);
    untilTargetPos(3, 500);
    autoDownUntil = pros::millis() + 1200;
    untilDelay(325, true);
    shouldClamp = false;
    claw.piston.set_value(false);

    // Go to the match loader using the right wall for a fresh Y reference.
    resetFromWall(WallSensor::Left, FieldWall::Bottom);
    resumeDrive();
    setMoveSpeed(400);
    setDriveMode(Direction::Reverse);
    setTargetPos(5 * 24 + 6, 24);
    untilTargetPos(3, 1000);
    autoLiftUntil = pros::millis() + 100;
    setDriveMode(Direction::Forward);
    setTargetPos(5 * 24 + 8, 0);
    setMoveSpeed(0);
    untilTargetH(3, 1000);

    disableTask(chassisTaskControl);
    disableTask(autoClampTaskControl);
    cascadeHold = false;
    drive.stop();
}

void (*selectedAuton)() = exampleIdle;
