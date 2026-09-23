#include "main.h"
#include "teleop.hpp"

#ifndef ENABLE_PID_TUNER
#define ENABLE_PID_TUNER 1 // Set to 0 to compile out the controller tuner.
#endif
#if ENABLE_PID_TUNER
#include "pid_tuner.hpp"
#endif

void initialize() {
    drive.initialize();
    cascade.initialize();
    drive.stop();
    cascade.hold();
    cascade.moveCascadeVoltage(0);

    drive.imu.reset(false);
    const uint32_t started = pros::millis();
    pros::delay(20);
    while (drive.imu.is_calibrating() && pros::millis() - started < 5000) {
        pros::delay(10);
    }

    if (drive.imu.is_calibrating() || !drive.sample().valid) {
        controller.print(0, 0, "Initialization failed");
    }

    odometry_task = new pros::Task(odometryTask, TASK_PRIORITY_DEFAULT + 3, TASK_STACK_DEPTH_DEFAULT, "odometry");
    chassis_task = new pros::Task(chassisTask, TASK_PRIORITY_DEFAULT + 2, TASK_STACK_DEPTH_DEFAULT, "chassis");
    cascade_task = new pros::Task(cascadeTask, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "cascade");
    clamp_task = new pros::Task(autoClampTask, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "clamp");
    print_task = new pros::Task(printTask, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT, "print");
    enableTask(odometryTaskControl);
    enableTask(printTaskControl);
}

void disabled() {
    disableTask(chassisTaskControl);
    drive.stop();
    disableTask(cascadeTaskControl);
    disableTask(autoClampTaskControl);
    autoLiftUntil = 0;
    autoDownUntil = 0;
    cascadeSpeed = 0;
    cascade.moveCascadeVoltage(0);
    disableTask(printTaskControl);
}

void competition_initialize() {
    disabled();
}

void autonomous() {
    disableTask(cascadeTaskControl);
    autoLiftUntil = 0;
    autoDownUntil = 0;
    cascadeSpeed = 0;
    shouldClamp = hasClamped.load();
    maxMoveSpeed = 600;
    minMoveSpeed = 0;
    maxTurnSpeed = 600;
    maxMotorSpeed = 600;
    driveMode = Direction::Auto;
    driveDisabled = false;
    const Pose pose = getRobotPose();
    setPos(pose.x, pose.y, pose.heading);
    drive.brake(true);
    enableTask(chassisTaskControl);
    enableTask(cascadeTaskControl);
    enableTask(autoClampTaskControl);
    enableTask(printTaskControl);

    selectedAuton();

    disableTask(chassisTaskControl);
    drive.stop();
    disableTask(autoClampTaskControl);
    disableTask(cascadeTaskControl);
    autoLiftUntil = 0;
    autoDownUntil = 0;
    cascadeSpeed = 0;
    cascade.moveCascadeVoltage(0);
}

void startAuton() {
    if (controller.get_digital(DIGITAL_A) && !AMode) {
        //autonomous();
        enableTask(chassisTaskControl);
        // exampleStraight();
        rightSideToggle();
        drive.brake(false);
        enableTask(cascadeTaskControl);
    }
}

void opcontrol() {
    disableTask(chassisTaskControl);
    drive.stop();
    drive.brake(false);
    disableTask(autoClampTaskControl);
    enableTask(cascadeTaskControl);
    enableTask(printTaskControl);
    enableTask(odometryTaskControl);
    updateButtons();

    while (true) {
        if (controller.get_digital(DIGITAL_Y) && !YMode) {
            runPidTuner();
            updateButtons();
        }
        tankBasic();
        cascadeCode();
        pistonClawCode();
        startAuton();
        updateButtons();
        pros::delay(20);
    }
}
