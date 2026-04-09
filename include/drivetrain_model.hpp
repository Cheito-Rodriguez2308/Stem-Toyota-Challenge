#pragma once

/*
 * Purpose:
 *   Declares the reduced-order differential-drive plant model used by the
 *   simulator.
 *
 * Role In The Architecture:
 *   This file belongs to the simulation layer. It advances the true robot
 *   state from commanded voltages while remaining independent of sensor and
 *   controller implementation details.
 *
 * Key Class:
 *   - DifferentialDriveDynamics
 *
 * Dependencies:
 *   - simulation_modules.hpp for the DynamicsModel interface
 */

#include "simulation_modules.hpp"

namespace robot_sim {

/**
 * Simplified differential-drive dynamics model.
 *
 * Layer:
 *   Simulation / plant dynamics.
 *
 * Assumptions:
 *   - The robot is a rigid planar differential-drive vehicle.
 *   - Left and right wheel speeds respond as first-order systems to commanded
 *     voltage-scaled free speed.
 *   - Drag and rolling resistance are modeled as lumped scalar effects.
 *   - Battery sag is approximated algebraically rather than through battery
 *     chemistry or thermal state.
 *
 * Not Modeled:
 *   - Wheel slip state and contact patch mechanics
 *   - Lateral scrub forces
 *   - Chassis compliance
 *   - Detailed motor winding dynamics
 *   - Collision or impact physics
 *
 * Usage:
 *   Construct once with DriveParams, call reset() for each scenario, then call
 *   step() at a fixed time step.
 */
class DifferentialDriveDynamics final : public DynamicsModel {
public:
    explicit DifferentialDriveDynamics(DriveParams drive_params);

    /// Stores scenario-specific modifiers before stepping begins.
    void reset(const ScenarioParams& scenario, const RobotState& initial_state) override;

    /**
     * Advances the true robot state by one fixed step.
     *
     * Input:
     *   - time_s: current simulation time [s].
     *   - time_step_s: integration step [s].
     *   - state: current true robot state.
     *   - input: commanded voltages and lumped disturbances.
     *
     * Output:
     *   Updated RobotState at time_s + time_step_s.
     */
    RobotState step(
        double time_s,
        double time_step_s,
        const RobotState& state,
        const RobotInput& input) override;

private:
    /**
     * Time derivatives of the state components integrated by the model.
     *
     * Units:
     *   - x_dot, y_dot: [m/s]
     *   - theta_dot: [rad/s]
     *   - *_velocity_dot: [m/s^2]
     *   - *_position_dot: [m/s]
     */
    struct Derivatives {
        double x_dot{0.0};
        double y_dot{0.0};
        double theta_dot{0.0};
        double left_velocity_dot{0.0};
        double right_velocity_dot{0.0};
        double left_position_dot{0.0};
        double right_position_dot{0.0};
    };

    /**
     * Scenario-adjusted parameters cached for one run.
     *
     * Why:
     *   Scenario overrides such as wheel mismatch or battery-sag forcing are
     *   applied once here so the derivative code stays focused on the equations.
     */
    struct EffectiveDriveParams {
        double mass_kg{15.0};
        double left_wheel_radius_m{0.045};
        double right_wheel_radius_m{0.045};
        double track_width_m{0.30};
        double left_free_speed_mps{0.0};
        double right_free_speed_mps{0.0};
        double nominal_battery_voltage_v{12.0};
        double min_battery_voltage_v{8.0};
        double wheel_velocity_response_time_s{0.25};
        double drivetrain_drag_n_per_mps{0.0};
        double rolling_resistance_n{0.0};
        double max_acceleration_mps2{4.0};
        double battery_internal_resistance_ohm{0.015};
        double left_drive_gain_scale{1.0};
        double right_drive_gain_scale{1.0};
        bool enable_battery_sag{true};
        bool enable_acceleration_saturation{true};
    };

    DriveParams base_drive_params_{};
    ScenarioParams scenario_{};

    /// Applies scenario modifiers to the stored base drivetrain parameters.
    EffectiveDriveParams effective_params() const;

    /// Computes instantaneous state derivatives from the current state and input.
    Derivatives compute_derivatives(
        const RobotState& state,
        const RobotInput& input,
        const EffectiveDriveParams& params) const;

    /// Computes one side's wheel acceleration [m/s^2].
    double compute_wheel_acceleration(
        double current_velocity_mps,
        double commanded_voltage_v,
        double free_speed_mps,
        double side_gain_scale,
        const EffectiveDriveParams& params) const;

    /// Integrates one step using explicit Euler.
    RobotState integrate_euler(
        double time_step_s,
        const RobotState& state,
        const RobotInput& input,
        const EffectiveDriveParams& params) const;

    /// Integrates one step using classical RK4.
    RobotState integrate_rk4(
        double time_step_s,
        const RobotState& state,
        const RobotInput& input,
        const EffectiveDriveParams& params) const;

    /// Applies a derivative increment to a state for Euler or RK4 staging.
    RobotState apply_derivatives(
        const RobotState& state,
        const Derivatives& derivatives,
        double scale,
        const EffectiveDriveParams& params) const;

    /// Estimates available battery voltage under load [V].
    double compute_battery_voltage(
        const RobotInput& input,
        const EffectiveDriveParams& params) const;
};

}  // namespace robot_sim
