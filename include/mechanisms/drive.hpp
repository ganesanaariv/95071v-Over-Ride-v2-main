#pragma once

#include "api.h"
#include "auton/config.hpp"

class Drivetrain {
    pros::Motor frontLeft, middleLeft, backLeft, frontRight, middleRight, backRight;
    pros::Distance front, back, right, left;
    bool holding = false;

public:
    pros::Imu imu;

    Drivetrain(std::array<int8_t, 3> leftPorts, std::array<int8_t, 3> rightPorts, uint8_t imuPort,
               std::array<uint8_t, 4> distancePorts)
        : frontLeft(leftPorts[0], pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::counts),
          middleLeft(leftPorts[1], pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::counts),
          backLeft(leftPorts[2], pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::counts),
          frontRight(rightPorts[0], pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::counts),
          middleRight(rightPorts[1], pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::counts),
          backRight(rightPorts[2], pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::counts),
          front(distancePorts[0]), back(distancePorts[1]), right(distancePorts[2]), left(distancePorts[3]),
          imu(imuPort) {}

    void initialize() {
        for (auto* motor : {&frontLeft, &middleLeft, &backLeft, &frontRight, &middleRight, &backRight}) {
            motor->set_voltage_limit(12000);
            motor->set_current_limit(2500);
        }
        brake(true);
        imu.set_data_rate(10);
    }
    SensorSample sample() {
        const double fl = frontLeft.get_position(), bl = backLeft.get_position();
        const double fr = frontRight.get_position(), br = backRight.get_position();
        const double heading = imu.get_rotation();
        const bool valid = !imu.is_calibrating() && std::isfinite(fl) && std::isfinite(bl) && std::isfinite(fr) &&
                           std::isfinite(br) && std::isfinite(heading);
        return {-(fl + bl) / 2, -(fr + br) / 2, heading, valid};
    }
    WallReading wallReading(WallSensor sensor) {
        pros::Distance* device = &front;
        switch (sensor) {
        case WallSensor::Front:
            break;
        case WallSensor::Back:
            device = &back;
            break;
        case WallSensor::Right:
            device = &right;
            break;
        case WallSensor::Left:
            device = &left;
            break;
        }
        return {static_cast<double>(device->get_distance()), device->get_confidence()};
    }
    void voltage(Output power) {
        const int l = static_cast<int>(limit(power.left, 600) * -20);
        const int r = static_cast<int>(limit(power.right, 600) * -20);
        frontLeft.move_voltage(l);
        middleLeft.move_voltage(l);
        backLeft.move_voltage(l);
        frontRight.move_voltage(r);
        middleRight.move_voltage(r);
        backRight.move_voltage(r);
    }
    void velocity(double leftRpm, double rightRpm) {
        leftRpm = limit(leftRpm, 600);
        rightRpm = limit(rightRpm, 600);
        frontLeft.move_velocity(-leftRpm);
        middleLeft.move_velocity(-leftRpm / 3);
        backLeft.move_velocity(-leftRpm);
        frontRight.move_velocity(-rightRpm);
        middleRight.move_velocity(-rightRpm / 3);
        backRight.move_velocity(-rightRpm);
    }
    void stop() {
        voltage({});
    }

    void brake(bool hold) {
        if (holding == hold) return;
        holding = hold;
        for (auto* motor : {&frontLeft, &middleLeft, &backLeft, &frontRight, &middleRight, &backRight})
            motor->set_brake_mode(hold ? pros::E_MOTOR_BRAKE_HOLD : pros::E_MOTOR_BRAKE_COAST);
    }
};
