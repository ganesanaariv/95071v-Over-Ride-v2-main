#pragma once

#include "config.hpp"

struct MotionOptions {
    uint32_t timeout = 0; // No deadline unless the auton supplies one.
    Direction direction = Direction::Default;
    TurnDirection turn = TurnDirection::Shortest;
    MotionMode mode = MotionMode::Stop;
    double minimumPower = 0, handoffDistance = 3;
    double lead = config.motion.poseLeadRatio;
    Arrival arrival = config.motion.arrival;
};
MotionOptions chainOptions(double power = 150, double distance = 3) {
    MotionOptions options;
    options.mode = MotionMode::Chain;
    options.minimumPower = power;
    options.handoffDistance = distance;
    return options;
}
struct MotionRequest {
    MotionKind kind = MotionKind::Point;
    Pose target{};
    double distance = 0;
    MotionOptions options{};
};

class PD {
    double previous = 0, derivative = 0;
    bool initialized = false;

public:
    Gains gains;
    explicit PD(Gains settings) : gains(settings) {}
    void reset() {
        initialized = false;
        derivative = 0;
    }
    double update(double error, double dt, bool angular = false) {
        if (dt <= 0) return gains.p * error;
        const double change = angular ? wrap(error - previous) : error - previous;
        const double rate = initialized ? change / dt : 0;
        derivative += dt / (gains.derivativeFilterSeconds + dt) * (rate - derivative);
        previous = error;
        initialized = true;
        return gains.p * error + gains.d * derivative;
    }
};

class SettlingWindow {
    bool inside = false;
    uint32_t since = 0, last = 0;

public:
    void reset() {
        inside = false;
    }
    bool update(bool acceptable, uint32_t now, uint32_t dwell, uint32_t maximumGap = 50) {
        if (!acceptable) {
            reset();
            return false;
        }
        if (!inside || elapsed(now, last, maximumGap + 1)) {
            since = now;
            inside = true;
        }
        last = now;
        return elapsed(now, since, dwell);
    }
};

double slewPower(double wanted, double previous, double rate, double dt) {
    if (wanted * previous < 0) return 0;
    if (std::fabs(wanted) <= std::fabs(previous)) return wanted;
    return sign(wanted) * std::min(std::fabs(wanted), std::fabs(previous) + rate * dt);
}
Output mix(double translation, double turn, double motorLimit) {
    turn = limit(turn, motorLimit);
    translation = limit(translation, motorLimit - std::fabs(turn));
    return {translation + turn, translation - turn};
}
double directedHeading(double current, double desired, TurnDirection direction) {
    double change = wrap(desired - current);
    if (std::fabs(change) == 180) change = 180;
    if (direction == TurnDirection::Clockwise && change < 0) change += 360;
    if (direction == TurnDirection::Counterclockwise && change > 0) change -= 360;
    return current + change;
}

class Motion {
    const Config& cfg;
    SettlingWindow settling;
    MotionRequest command{};
    uint32_t started = 0;
    double direction = 1, turnTarget = 0, startTravel = 0;
    double previousPower = 0, previousTurn = 0;
    Vec terminalAxis{};
    bool terminal = false, aligning = false, recovering = false;
    unsigned reapproaches = 0;

public:
    PD translation, turning, drivingHeading;
    Result result = Result::Idle;
    double positionError = 0, headingError = 0;
    explicit Motion(const Config& settings = config)
        : cfg(settings), translation(cfg.motion.translation), turning(cfg.motion.turn),
          drivingHeading(cfg.motion.drivingHeading) {}
    const MotionRequest& request() const {
        return command;
    }
    double capturedDirection() const {
        return direction;
    }
    bool active() const {
        return result == Result::Running || result == Result::HandedOff;
    }
    void stop(Result reason = Result::Idle) {
        result = reason;
        resetHistory();
    }
    void resetHistory() {
        translation.reset();
        turning.reset();
        drivingHeading.reset();
        settling.reset();
        previousPower = previousTurn = 0;
    }
    double distanceRemaining(const Estimate& estimate) const {
        return command.distance - (estimate.travel - startTravel);
    }
    void setArrival(Arrival arrival) {
        command.options.arrival = arrival;
        settling.reset();
    }

    void start(MotionRequest next, const Estimate& estimate, uint32_t now) {
        const double carry = active() && command.options.mode == MotionMode::Chain ? previousPower : 0;
        command = next;
        started = now;
        startTravel = estimate.travel;
        terminal = aligning = recovering = false;
        reapproaches = 0;
        resetHistory();
        result = Result::Running;
        const auto& o = command.options;
        if (!finite(command.target) || !std::isfinite(command.distance) || !std::isfinite(o.lead) || o.lead < 0 ||
            o.lead > 1 || !std::isfinite(o.minimumPower) || o.minimumPower < 0 || o.minimumPower > 600 ||
            !std::isfinite(o.handoffDistance) || o.handoffDistance < 0 || !validArrival(o.arrival) ||
            (o.mode == MotionMode::Chain && command.kind != MotionKind::Point && command.kind != MotionKind::Pose)) {
            result = Result::Invalid;
            return;
        }
        const Vec delta = position(command.target) - position(estimate.pose);
        direction = o.direction == Direction::Reverse ? -1 : 1;
        if ((o.direction == Direction::Auto || o.direction == Direction::Default) && delta.norm() > 1e-6)
            direction = std::fabs(wrap(bearing(delta.x, delta.y) - estimate.pose.heading)) > 90 ? -1 : 1;
        if (command.kind == MotionKind::Distance) direction = sign(command.distance);
        turnTarget = directedHeading(estimate.pose.heading, command.target.heading, o.turn);
        terminalAxis = delta.norm() > 1e-6 ? delta * (1 / delta.norm()) : forward(estimate.pose.heading) * direction;
        if (carry * direction > 0) previousPower = carry;
    }

    bool arrived(const Estimate& estimate, Arrival arrival) const {
        if (!estimate.valid || estimate.velocity.norm() > arrival.linearSpeedInPerSec ||
            std::fabs(estimate.angularRate) > arrival.angularSpeedDegPerSec)
            return false;
        const bool headingOK =
            command.kind == MotionKind::Turn
                ? std::fabs(turnTarget - estimate.pose.heading) <= arrival.headingDeg
                : std::fabs(wrap(command.target.heading - estimate.pose.heading)) <= arrival.headingDeg;
        if (command.kind == MotionKind::Turn) return headingOK;
        if (command.kind == MotionKind::Distance)
            return std::fabs(command.distance - (estimate.travel - startTravel)) <= arrival.positionIn && headingOK;
        return (position(command.target) - position(estimate.pose)).norm() <= arrival.positionIn &&
               (command.kind != MotionKind::Pose || headingOK);
    }

    Output update(const Estimate& estimate, uint32_t now, double dt, double moveCap, double turnCap, double motorCap,
                  bool paused, double minimumMovePower = 0) {
        if (!active()) return {};
        if (command.options.timeout && elapsed(now, started, command.options.timeout)) {
            stop(Result::TimedOut);
            return {};
        }
        if (!estimate.valid || elapsed(now, estimate.sampledAt, cfg.staleMs + 1)) {
            stop(Result::SensorFault);
            return {};
        }
        if (paused) {
            resetHistory();
            return {};
        }
        double power = 0, turn = 0;
        const auto& arrival = command.options.arrival;
        const Vec delta = position(command.target) - position(estimate.pose);
        positionError = delta.norm();
        if (command.kind == MotionKind::Turn) {
            headingError = turnTarget - estimate.pose.heading;
            turn = turning.update(headingError, dt);
            if (std::fabs(headingError) <= arrival.headingDeg) turn = 0;
        } else if (command.kind == MotionKind::Distance) {
            const double error = command.distance - (estimate.travel - startTravel);
            positionError = std::fabs(error);
            headingError = wrap(command.target.heading - estimate.pose.heading);
            power = translation.update(error, dt);
            turn = drivingHeading.update(headingError, dt, true);
            if (positionError <= arrival.positionIn) power = 0;
        } else {
            const bool poseMove = command.kind == MotionKind::Pose;
            const double previousCross = delta.x * terminalAxis.y - delta.y * terminalAxis.x;
            if (!terminal && positionError < cfg.motion.terminalDistanceIn &&
                (!recovering || std::fabs(previousCross) < arrival.positionIn * 0.5 ||
                 positionError <= arrival.positionIn)) {
                terminal = true;
                recovering = false;
                terminalAxis = poseMove ? forward(command.target.heading) * direction
                                        : (positionError > 1e-6 ? delta * (1 / positionError)
                                                                : forward(estimate.pose.heading) * direction);
                translation.reset();
                drivingHeading.reset();
            }
            const double along = delta.dot(terminalAxis);
            const double cross = delta.x * terminalAxis.y - delta.y * terminalAxis.x;
            const bool sidewaysAtTarget = std::fabs(along) < arrival.positionIn * 0.5 &&
                                          positionError > arrival.positionIn &&
                                          std::fabs(cross) > arrival.positionIn * 0.5;
            if (terminal && (sidewaysAtTarget ||
                             std::fabs(cross) > std::max(cfg.motion.crossTrackExitIn, arrival.positionIn * 1.5) ||
                             (aligning && positionError > arrival.positionIn * 1.5))) {
                if (++reapproaches > cfg.motion.maximumReapproaches) {
                    stop(Result::Stalled);
                    return {};
                }
                terminal = aligning = false;
                recovering = true;
                translation.reset();
                drivingHeading.reset();
            }
            Vec aim = delta;
            if (poseMove && !terminal)
                aim = delta - forward(command.target.heading) * (direction * command.options.lead * positionError);
            const double wantedHeading = terminal ? bearing(terminalAxis.x, terminalAxis.y) + (direction < 0 ? 180 : 0)
                                                  : bearing(aim.x, aim.y) + (direction < 0 ? 180 : 0);
            headingError = wrap(wantedHeading - estimate.pose.heading);
            const double error = terminal ? along * direction : positionError * direction;
            power = translation.update(error, dt) * std::max(0.0, std::cos(headingError * radians));
            turn = drivingHeading.update(headingError, dt, true);
            if (terminal) power = limit(power, cfg.motion.terminalPower);
            if (poseMove) {
                const double curveCap =
                    cfg.motion.curvePower / std::max(0.25, std::fabs(std::sin(headingError * radians)));
                power = limit(power, curveCap);
            }
            if (command.options.mode == MotionMode::Stop && positionError <= arrival.positionIn) {
                power = 0;
                if (poseMove) {
                    if (!aligning) {
                        turning.reset();
                        aligning = true;
                    }
                    headingError = wrap(command.target.heading - estimate.pose.heading);
                    turn = std::fabs(headingError) <= arrival.headingDeg ? 0 : turning.update(headingError, dt, true);
                } else
                    turn = 0;
            }
            if (command.options.mode == MotionMode::Chain) {
                if (result != Result::HandedOff && positionError <= command.options.handoffDistance) {
                    result = Result::HandedOff;
                }
                if (std::fabs(headingError) < 45 && power * direction >= 0)
                    power = direction * std::max({std::fabs(power), command.options.minimumPower, minimumMovePower});
            }
        }
        if (command.options.mode == MotionMode::Stop &&
            settling.update(arrived(estimate, arrival), now, arrival.dwellMs, cfg.staleMs)) {
            stop(Result::Settled);
            return {};
        }
        const double minimum = std::clamp(std::max(minimumMovePower, command.options.minimumPower), 0.0, moveCap);
        if (command.options.mode == MotionMode::Stop && command.kind != MotionKind::Turn &&
            positionError > arrival.positionIn && std::fabs(headingError) < 45 && power != 0)
            power = sign(power) * std::max(std::fabs(power), minimum);
        power = slewPower(limit(power, moveCap), previousPower, cfg.motion.accelerationPowerPerSec, dt);
        turn = slewPower(limit(turn, turnCap), previousTurn, cfg.motion.turnPowerPerSec, dt);
        const Output output = mix(power, turn, motorCap);
        previousPower = (output.left + output.right) / 2;
        previousTurn = (output.left - output.right) / 2;
        return output;
    }
};
