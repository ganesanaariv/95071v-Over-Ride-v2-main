#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#define TASK_PRIORITY_DEFAULT 8
#define TASK_STACK_DEPTH_DEFAULT 8192

struct FakeHardware {
    std::atomic<bool> disabled{false}, autonomous{false}, connected{true}, sensorValid{true};
    std::array<std::atomic<int>, 22> voltage{}, rpm{};
    std::array<std::atomic<bool>, 256> piston{};
    std::atomic<int> acquisitionDistance{1000};
    std::atomic<int> wallReadCount{0};
    std::atomic<int> printDelay{0}, printCount{0}, samples{0};
    std::atomic<double> encoderPosition{0}, rotation{0};
    std::array<std::atomic<int>, 2> sticks{};
    std::array<std::atomic<bool>, 12> buttons{}, previousButtons{};
    std::mutex screenMutex;
    std::array<std::string, 3> screen{};
};
FakeHardware fake;

namespace pros {
const auto epoch = std::chrono::steady_clock::now();
uint32_t millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - epoch).count();
}
uint64_t micros() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - epoch).count();
}
void delay(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
using Mutex = std::mutex;
class Task {
public:
    template <typename F> Task(F function, uint32_t, uint32_t, const char*) {
        std::thread(function).detach();
    }
};
namespace competition {
bool is_disabled() {
    return fake.disabled.load();
}
bool is_autonomous() {
    return fake.autonomous.load();
}
} // namespace competition
namespace v5 {
enum class MotorGears { blue };
enum class MotorEncoderUnits { counts };
} // namespace v5
enum motor_brake_mode_e_t { E_MOTOR_BRAKE_HOLD, E_MOTOR_BRAKE_COAST };
class Motor {
    int port;

public:
    Motor(int8_t number, v5::MotorGears, v5::MotorEncoderUnits = v5::MotorEncoderUnits::counts)
        : port(std::abs(number)) {}
    void set_voltage_limit(int) {}
    void set_current_limit(int) {}
    void set_brake_mode(motor_brake_mode_e_t) {}
    void move_voltage(int voltage) {
        fake.voltage[port] = voltage;
    }
    void move_velocity(int velocity) {
        fake.rpm[port] = velocity;
    }
    double get_position() {
        ++fake.samples;
        return fake.sensorValid ? fake.encoderPosition.load() : std::numeric_limits<double>::infinity();
    }
};
class Imu {
public:
    explicit Imu(int) {}
    void set_data_rate(int) {}
    void reset(bool) {}
    bool is_calibrating() {
        return false;
    }
    double get_rotation() {
        return fake.rotation.load();
    }
};
class Distance {
    int port;

public:
    explicit Distance(int number) : port(number) {}
    int get_distance() {
        if (port == 12) return fake.acquisitionDistance.load();
        ++fake.wallReadCount;
        return 500;
    }
    int get_confidence() {
        return 63;
    }
};
namespace adi {
class DigitalOut {
    uint8_t port;

public:
    explicit DigitalOut(uint8_t number) : port(number) {}
    void set_value(bool value) {
        fake.piston[port] = value;
    }
};
} // namespace adi
enum controller_id_e_t { E_CONTROLLER_MASTER };
enum controller_analog_e_t { E_CONTROLLER_ANALOG_LEFT_Y, E_CONTROLLER_ANALOG_RIGHT_Y };
enum controller_digital_e_t {
    E_CONTROLLER_DIGITAL_L1,
    E_CONTROLLER_DIGITAL_L2,
    E_CONTROLLER_DIGITAL_R1,
    E_CONTROLLER_DIGITAL_R2,
    E_CONTROLLER_DIGITAL_A,
    E_CONTROLLER_DIGITAL_B,
    E_CONTROLLER_DIGITAL_X,
    E_CONTROLLER_DIGITAL_Y,
    E_CONTROLLER_DIGITAL_UP,
    E_CONTROLLER_DIGITAL_DOWN,
    E_CONTROLLER_DIGITAL_LEFT,
    E_CONTROLLER_DIGITAL_RIGHT
};
class Controller {
public:
    explicit Controller(controller_id_e_t) {}
    template <typename... Args> void print(int line, int, const char* format, Args... args) {
        char text[128];
        if constexpr (sizeof...(args) == 0)
            std::snprintf(text, sizeof(text), "%s", format);
        else
            std::snprintf(text, sizeof(text), format, args...);
        {
            std::lock_guard<std::mutex> lock(fake.screenMutex);
            fake.screen[line] = text;
        }
        ++fake.printCount;
        delay(fake.printDelay.load());
    }
    int get_analog(controller_analog_e_t input) {
        return fake.sticks[input];
    }
    bool get_digital(controller_digital_e_t input) {
        return fake.buttons[input];
    }
    bool get_digital_new_press(controller_digital_e_t input) {
        const bool down = fake.buttons[input];
        const bool previous = fake.previousButtons[input].exchange(down);
        return down && !previous;
    }
    bool is_connected() {
        return fake.connected.load();
    }
};
} // namespace pros
