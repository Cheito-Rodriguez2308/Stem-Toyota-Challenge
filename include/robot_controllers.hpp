#pragma once

/*
 * Purpose:
 *   Declares task-level robot controllers built from the low-level control
 *   primitives.
 *
 * Role In The Architecture:
 *   This file belongs to the control layer. It translates estimated robot
 *   state and scenario goals into RobotInput voltage commands while remaining
 *   independent of the drivetrain integration and sensor error logic.
 *
 * Key Classes:
 *   - PointToPointController
 *   - TurnInPlaceController
 *   - make_controller
 *
 * Dependencies:
 *   - control_primitives.hpp for PID, slew limiting, and feedforward
 *   - simulation_modules.hpp for the Controller interface
 */

#include "control_primitives.hpp"
#include "simulation_modules.hpp"

#include <memory>

namespace robot_sim {

/**
 * Drives the robot toward a target position and final heading.
 *
 * Layer:
 *   Control / autonomous behavior.
 *
 * Assumptions:
 *   - The controller operates on planar estimated state.
 *   - Position control is expressed as a distance loop plus heading loop.
 *   - Output is left/right voltage command, not wheel torque or current.
 *
 * Usage:
 *   Best suited for short autonomous moves and scenario validation where a
 *   minimal practical controller is preferred over trajectory optimization.
 */
class PointToPointController final : public Controller {
public:
    PointToPointController(DriveParams drive_params, ControllerParams controller_params);

    void reset(const ScenarioParams& scenario, const RobotState& initial_state) override;

    /// Computes left/right drive voltage commands [V] from the current estimate.
    RobotInput compute_command(
        double time_s,
        const RobotState& true_state,
        const EstimatedState& estimated_state,
        const SensorReadings& sensors) override;

private:
    DriveParams drive_params_{};
    ControllerParams controller_params_{};
    ScenarioParams scenario_{};
    PIDController distance_pid_{};
    PIDController heading_pid_{};
    SlewRateLimiter left_limiter_{};
    SlewRateLimiter right_limiter_{};
    SimpleFeedforward feedforward_{};
    double previous_time_s_{0.0};
    bool has_previous_time_{false};
};

/**
 * Rotates the robot to a desired heading with zero nominal forward command.
 *
 * Layer:
 *   Control / autonomous behavior.
 *
 * Usage:
 *   Used for in-place turns and scenarios where heading alignment is the only
 *   goal.
 */
class TurnInPlaceController final : public Controller {
public:
    TurnInPlaceController(DriveParams drive_params, ControllerParams controller_params);

    void reset(const ScenarioParams& scenario, const RobotState& initial_state) override;

    /// Computes equal-and-opposite voltage commands [V] for a turn-in-place maneuver.
    RobotInput compute_command(
        double time_s,
        const RobotState& true_state,
        const EstimatedState& estimated_state,
        const SensorReadings& sensors) override;

private:
    DriveParams drive_params_{};
    ControllerParams controller_params_{};
    ScenarioParams scenario_{};
    PIDController heading_pid_{};
    SlewRateLimiter left_limiter_{};
    SlewRateLimiter right_limiter_{};
    double previous_time_s_{0.0};
    bool has_previous_time_{false};
};

/// Factory that chooses a controller implementation from scenario metadata.
std::unique_ptr<Controller> make_controller(
    const ScenarioParams& scenario,
    const DriveParams& drive_params,
    const ControllerParams& controller_params);

}  // namespace robot_sim
