/*
 * Purpose:
 *   Implements reusable scalar control primitives.
 *
 * Role In The Architecture:
 *   This translation unit belongs to the control layer but stays free of
 *   scenario or drivetrain assumptions. Higher-level controllers assemble these
 *   primitives into robot-specific behavior.
 *
 * Key Functions:
 *   - PIDController::update
 *   - SlewRateLimiter::limit
 *   - SimpleFeedforward::calculate
 *
 * Dependencies:
 *   - control_primitives.hpp
 *   - math_utils.hpp for clamping and rate limiting
 */

#include "control_primitives.hpp"

#include "math_utils.hpp"

#include <stdexcept>

namespace robot_sim {

PIDController::PIDController(PIDGains gains)
    : gains_(gains) {}

void PIDController::set_gains(const PIDGains& gains) {
    gains_ = gains;
}

const PIDGains& PIDController::gains() const noexcept {
    return gains_;
}

void PIDController::reset() {
    integral_term_ = 0.0;
    previous_error_ = 0.0;
    last_output_ = 0.0;
    first_update_ = true;
}

double PIDController::update(double error, double time_step_s) {
    if (time_step_s <= 0.0) {
        throw std::invalid_argument("PIDController requires a positive time step.");
    }

    // Engineering Notes:
    //   This is the standard discrete PID form
    //     u = kp*e + ki*integral(e dt) + kd*de/dt
    //   with explicit integral clamping and output saturation.
    // Assumptions:
    //   - The caller uses a stable fixed or near-fixed control period.
    //   - Sensor noise is not so large that raw finite-difference derivative
    //     becomes unusable.
    // What Could Break In Reality:
    //   - Variable-rate control execution
    //   - Severe measurement noise or quantization
    //   - Actuator deadband, latency, or saturation interactions beyond the
    //     simple output clamp used here
    integral_term_ += error * time_step_s;
    integral_term_ = math::clamp(integral_term_, gains_.integral_min, gains_.integral_max);

    const double derivative = first_update_ ? 0.0 : (error - previous_error_) / time_step_s;
    const double raw_output =
        gains_.kp * error + gains_.ki * integral_term_ + gains_.kd * derivative;
    last_output_ = math::clamp(raw_output, gains_.output_min, gains_.output_max);
    previous_error_ = error;
    first_update_ = false;
    return last_output_;
}

double PIDController::update_from_setpoint(double setpoint, double measurement, double time_step_s) {
    return update(setpoint - measurement, time_step_s);
}

double PIDController::last_output() const noexcept {
    return last_output_;
}

double PIDController::integral_term() const noexcept {
    return integral_term_;
}

SlewRateLimiter::SlewRateLimiter(double max_rate_per_s)
    : max_rate_per_s_(max_rate_per_s) {}

void SlewRateLimiter::set_max_rate_per_s(double max_rate_per_s) {
    max_rate_per_s_ = max_rate_per_s;
}

void SlewRateLimiter::reset(double value) {
    previous_value_ = value;
    initialized_ = true;
}

double SlewRateLimiter::limit(double requested_value, double time_step_s) {
    if (!initialized_) {
        previous_value_ = requested_value;
        initialized_ = true;
        return previous_value_;
    }

    if (max_rate_per_s_ <= 0.0 || time_step_s <= 0.0) {
        previous_value_ = requested_value;
        return previous_value_;
    }

    const double max_delta = max_rate_per_s_ * time_step_s;
    previous_value_ = math::saturate_rate(requested_value, previous_value_, max_delta);
    return previous_value_;
}

double SlewRateLimiter::last_value() const noexcept {
    return previous_value_;
}

SimpleFeedforward::SimpleFeedforward(double kv, double ka)
    : kv_(kv), ka_(ka) {}

double SimpleFeedforward::calculate(double target_velocity, double target_acceleration) const {
    return kv_ * target_velocity + ka_ * target_acceleration;
}

}  // namespace robot_sim
