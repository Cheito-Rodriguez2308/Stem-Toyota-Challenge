#pragma once

/*
 * Purpose:
 *   Declares reusable low-level control primitives used by the robot
 *   controllers.
 *
 * Role In The Architecture:
 *   This file belongs to the control layer but stays independent of any
 *   specific robot task. Higher-level controllers compose these primitives to
 *   produce voltage commands without embedding the primitive math repeatedly.
 *
 * Key Classes:
 *   - PIDController
 *   - SlewRateLimiter
 *   - SimpleFeedforward
 *
 * Dependencies:
 *   - core_types.hpp for PIDGains
 */

#include "core_types.hpp"

namespace robot_sim {

/**
 * Scalar PID controller with integral clamping and output saturation.
 *
 * Layer:
 *   Control primitives.
 *
 * Assumptions:
 *   - The caller provides a positive fixed or quasi-fixed time step [s].
 *   - Gains are tuned for the error and output units used by the caller.
 *
 * Usage:
 *   Call reset() when starting a new run, then call update() once per control
 *   step with the current error.
 */
class PIDController {
public:
    explicit PIDController(PIDGains gains = {});

    /// Replaces the controller gains used on subsequent updates.
    void set_gains(const PIDGains& gains);

    /// Returns the currently configured gains.
    const PIDGains& gains() const noexcept;

    /// Clears integral, derivative, and cached output state.
    void reset();

    /**
     * Updates the controller from a direct error signal.
     *
     * Input:
     *   - error: control error in the caller's native units.
     *   - time_step_s: controller update period [s].
     *
     * Output:
     *   Saturated controller output, typically interpreted as voltage [V].
     */
    double update(double error, double time_step_s);

    /**
     * Convenience wrapper that computes error = setpoint - measurement.
     */
    double update_from_setpoint(double setpoint, double measurement, double time_step_s);

    /// Returns the most recently issued output.
    double last_output() const noexcept;

    /// Returns the current integral accumulator.
    double integral_term() const noexcept;

private:
    PIDGains gains_{};
    double integral_term_{0.0};
    double previous_error_{0.0};
    double last_output_{0.0};
    bool first_update_{true};
};

/**
 * Symmetric slew-rate limiter for scalar commands.
 *
 * Layer:
 *   Control primitives / actuator command shaping.
 *
 * Usage:
 *   Use this to soften abrupt voltage steps and keep controller commands closer
 *   to what a real actuator path could accept.
 */
class SlewRateLimiter {
public:
    explicit SlewRateLimiter(double max_rate_per_s = 0.0);

    /// Sets the maximum allowed change rate [units / s].
    void set_max_rate_per_s(double max_rate_per_s);

    /// Reinitializes the limiter to a known output value.
    void reset(double value = 0.0);

    /**
     * Applies the slew-rate limit.
     *
     * Input:
     *   - requested_value: desired output value.
     *   - time_step_s: time elapsed since the previous limit() call [s].
     */
    double limit(double requested_value, double time_step_s);

    /// Returns the most recently emitted value.
    double last_value() const noexcept;

private:
    double max_rate_per_s_{0.0};
    double previous_value_{0.0};
    bool initialized_{false};
};

/**
 * Minimal feedforward model based on desired velocity and acceleration.
 *
 * Layer:
 *   Control primitives.
 *
 * Assumptions:
 *   This is intentionally simple. It does not model static friction or detailed
 *   motor physics.
 */
class SimpleFeedforward {
public:
    SimpleFeedforward(double kv = 0.0, double ka = 0.0);

    /// Computes the feedforward contribution, typically interpreted as voltage [V].
    double calculate(double target_velocity, double target_acceleration) const;

private:
    double kv_{0.0}; ///< Velocity gain [V / (m/s)] or other caller-defined units.
    double ka_{0.0}; ///< Acceleration gain [V / (m/s^2)] or other caller-defined units.
};

}  // namespace robot_sim
