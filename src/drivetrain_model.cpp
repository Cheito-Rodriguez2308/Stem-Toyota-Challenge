/*
 * Purpose:
 *   Implements the reduced-order differential-drive plant model.
 *
 * Role In The Architecture:
 *   This translation unit advances the ground-truth robot state from control
 *   inputs. It remains intentionally separate from sensor simulation,
 *   estimation, and control logic.
 *
 * Key Functions:
 *   - DifferentialDriveDynamics::step
 *   - DifferentialDriveDynamics::compute_derivatives
 *   - DifferentialDriveDynamics::compute_wheel_acceleration
 *
 * Dependencies:
 *   - drivetrain_model.hpp
 *   - math_utils.hpp for clamping and angle wrapping
 */

#include "drivetrain_model.hpp"

#include "math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot_sim {
namespace {

/// Returns -1, 0, or +1 with a small dead zone around zero.
double sign_or_zero(double value) {
    if (value > 1e-6) {
        return 1.0;
    }
    if (value < -1e-6) {
        return -1.0;
    }
    return 0.0;
}

/// Saturates a requested voltage to the currently available bus limit [V].
double clamp_voltage(double requested_voltage, double max_magnitude_voltage) {
    return math::clamp(requested_voltage, -max_magnitude_voltage, max_magnitude_voltage);
}

}  // namespace

DifferentialDriveDynamics::DifferentialDriveDynamics(DriveParams drive_params)
    : base_drive_params_(std::move(drive_params)) {}

void DifferentialDriveDynamics::reset(const ScenarioParams& scenario, const RobotState& initial_state) {
    DynamicsModel::reset(scenario, initial_state);
    scenario_ = scenario;
}

RobotState DifferentialDriveDynamics::step(
    double time_s,
    double time_step_s,
    const RobotState& state,
    const RobotInput& input) {
    (void)time_s;

    const EffectiveDriveParams params = effective_params();
    RobotState next_state = scenario_.integrator == IntegratorType::RK4
        ? integrate_rk4(time_step_s, state, input, params)
        : integrate_euler(time_step_s, state, input, params);

    next_state.linear_velocity_mps =
        0.5 * (next_state.left_wheel_velocity_mps + next_state.right_wheel_velocity_mps);
    next_state.angular_velocity_radps =
        (next_state.right_wheel_velocity_mps - next_state.left_wheel_velocity_mps) /
        std::max(1e-6, params.track_width_m);
    next_state.pose.theta_rad = math::wrap_angle(next_state.pose.theta_rad);
    next_state.battery_voltage_v = compute_battery_voltage(input, params);
    next_state.sim_time_s = state.sim_time_s + time_step_s;
    return next_state;
}

DifferentialDriveDynamics::EffectiveDriveParams DifferentialDriveDynamics::effective_params() const {
    EffectiveDriveParams params{};
    params.mass_kg = std::max(1e-6, base_drive_params_.mass_kg);
    params.left_wheel_radius_m =
        base_drive_params_.wheel_radius_m *
        base_drive_params_.left_wheel_radius_scale *
        scenario_.left_wheel_radius_scale;
    params.right_wheel_radius_m =
        base_drive_params_.wheel_radius_m *
        base_drive_params_.right_wheel_radius_scale *
        scenario_.right_wheel_radius_scale;
    params.track_width_m = std::max(1e-6, base_drive_params_.track_width_m);
    params.left_free_speed_mps = base_drive_params_.motor.free_speed_radps * params.left_wheel_radius_m;
    params.right_free_speed_mps = base_drive_params_.motor.free_speed_radps * params.right_wheel_radius_m;
    params.nominal_battery_voltage_v = base_drive_params_.nominal_battery_voltage_v;
    params.min_battery_voltage_v = base_drive_params_.min_battery_voltage_v;
    params.wheel_velocity_response_time_s =
        std::max(1e-3, base_drive_params_.wheel_velocity_response_time_s);
    params.drivetrain_drag_n_per_mps = base_drive_params_.drivetrain_drag_n_per_mps;
    params.rolling_resistance_n = base_drive_params_.rolling_resistance_n;
    params.max_acceleration_mps2 = base_drive_params_.max_acceleration_mps2;
    params.battery_internal_resistance_ohm = base_drive_params_.battery_internal_resistance_ohm;
    params.left_drive_gain_scale =
        base_drive_params_.left_drive_gain_scale * scenario_.left_drive_scale;
    params.right_drive_gain_scale =
        base_drive_params_.right_drive_gain_scale * scenario_.right_drive_scale;
    params.enable_battery_sag =
        base_drive_params_.enable_battery_sag || scenario_.enable_battery_sag;
    params.enable_acceleration_saturation = base_drive_params_.enable_acceleration_saturation;
    return params;
}

DifferentialDriveDynamics::Derivatives DifferentialDriveDynamics::compute_derivatives(
    const RobotState& state,
    const RobotInput& input,
    const EffectiveDriveParams& params) const {
    const double available_voltage_v = params.enable_battery_sag
        ? std::max(params.min_battery_voltage_v, state.battery_voltage_v)
        : params.nominal_battery_voltage_v;

    const double left_command_v = clamp_voltage(input.left_voltage_command_v, available_voltage_v);
    const double right_command_v = clamp_voltage(input.right_voltage_command_v, available_voltage_v);

    double left_acceleration_mps2 = compute_wheel_acceleration(
        state.left_wheel_velocity_mps,
        left_command_v,
        params.left_free_speed_mps,
        params.left_drive_gain_scale,
        params);
    double right_acceleration_mps2 = compute_wheel_acceleration(
        state.right_wheel_velocity_mps,
        right_command_v,
        params.right_free_speed_mps,
        params.right_drive_gain_scale,
        params);

    const double external_linear_acceleration_mps2 =
        (input.external_force_x_n - scenario_.external_disturbance_force_n) / params.mass_kg;
    left_acceleration_mps2 += external_linear_acceleration_mps2;
    right_acceleration_mps2 += external_linear_acceleration_mps2;

    const double angular_velocity_radps =
        (state.right_wheel_velocity_mps - state.left_wheel_velocity_mps) / params.track_width_m;
    const double linear_velocity_mps =
        0.5 * (state.left_wheel_velocity_mps + state.right_wheel_velocity_mps);

    // Engineering Notes:
    //   Kinematics use the standard differential-drive approximation:
    //     v     = (v_r + v_l) / 2
    //     omega = (v_r - v_l) / track_width
    //     x_dot = v cos(theta)
    //     y_dot = v sin(theta)
    //     theta_dot = omega
    //   Wheel-speed dynamics are approximated as first-order relaxation toward a
    //   voltage-scaled free-speed target with simple drag and rolling resistance.
    // Assumptions:
    //   - Pure rolling in the longitudinal direction
    //   - No lateral slip state
    //   - Lumped drag and rolling resistance
    // What Could Break In Reality:
    //   - Aggressive turning with scrub or wheel slip
    //   - Contact impacts, terrain changes, or chassis compliance
    //   - Highly nonlinear motor/gearbox behavior not captured by the
    //     first-order response approximation
    Derivatives derivatives{};
    derivatives.x_dot = linear_velocity_mps * std::cos(state.pose.theta_rad);
    derivatives.y_dot = linear_velocity_mps * std::sin(state.pose.theta_rad);
    derivatives.theta_dot = angular_velocity_radps;
    derivatives.left_velocity_dot = left_acceleration_mps2;
    derivatives.right_velocity_dot = right_acceleration_mps2;
    derivatives.left_position_dot = state.left_wheel_velocity_mps;
    derivatives.right_position_dot = state.right_wheel_velocity_mps;
    return derivatives;
}

double DifferentialDriveDynamics::compute_wheel_acceleration(
    double current_velocity_mps,
    double commanded_voltage_v,
    double free_speed_mps,
    double side_gain_scale,
    const EffectiveDriveParams& params) const {
    const double nominal_voltage_v = std::max(1e-6, params.nominal_battery_voltage_v);
    const double target_velocity_mps =
        side_gain_scale * free_speed_mps * (commanded_voltage_v / nominal_voltage_v);

    double acceleration_mps2 =
        (target_velocity_mps - current_velocity_mps) / params.wheel_velocity_response_time_s;

    const double drag_acceleration_mps2 =
        params.drivetrain_drag_n_per_mps * current_velocity_mps / params.mass_kg;
    acceleration_mps2 -= drag_acceleration_mps2;

    const double effective_motion_sign =
        sign_or_zero(std::abs(current_velocity_mps) > 1e-5 ? current_velocity_mps : target_velocity_mps);
    if (effective_motion_sign != 0.0) {
        acceleration_mps2 -= effective_motion_sign * (params.rolling_resistance_n / params.mass_kg);
        if (scenario_.enable_push_load && scenario_.push_force_n > 0.0) {
            acceleration_mps2 -= effective_motion_sign * (scenario_.push_force_n / params.mass_kg);
        }
    }

    if (params.enable_acceleration_saturation) {
        acceleration_mps2 = math::clamp(
            acceleration_mps2,
            -params.max_acceleration_mps2,
            params.max_acceleration_mps2);
    }

    return acceleration_mps2;
}

RobotState DifferentialDriveDynamics::integrate_euler(
    double time_step_s,
    const RobotState& state,
    const RobotInput& input,
    const EffectiveDriveParams& params) const {
    const Derivatives derivatives = compute_derivatives(state, input, params);
    return apply_derivatives(state, derivatives, time_step_s, params);
}

RobotState DifferentialDriveDynamics::integrate_rk4(
    double time_step_s,
    const RobotState& state,
    const RobotInput& input,
    const EffectiveDriveParams& params) const {
    const Derivatives k1 = compute_derivatives(state, input, params);
    const RobotState state_2 = apply_derivatives(state, k1, 0.5 * time_step_s, params);
    const Derivatives k2 = compute_derivatives(state_2, input, params);
    const RobotState state_3 = apply_derivatives(state, k2, 0.5 * time_step_s, params);
    const Derivatives k3 = compute_derivatives(state_3, input, params);
    const RobotState state_4 = apply_derivatives(state, k3, time_step_s, params);
    const Derivatives k4 = compute_derivatives(state_4, input, params);

    RobotState next_state = state;
    next_state.pose.x_m +=
        time_step_s * (k1.x_dot + 2.0 * k2.x_dot + 2.0 * k3.x_dot + k4.x_dot) / 6.0;
    next_state.pose.y_m +=
        time_step_s * (k1.y_dot + 2.0 * k2.y_dot + 2.0 * k3.y_dot + k4.y_dot) / 6.0;
    next_state.pose.theta_rad +=
        time_step_s * (k1.theta_dot + 2.0 * k2.theta_dot + 2.0 * k3.theta_dot + k4.theta_dot) / 6.0;
    next_state.left_wheel_velocity_mps +=
        time_step_s *
        (k1.left_velocity_dot + 2.0 * k2.left_velocity_dot + 2.0 * k3.left_velocity_dot + k4.left_velocity_dot) /
        6.0;
    next_state.right_wheel_velocity_mps +=
        time_step_s *
        (k1.right_velocity_dot + 2.0 * k2.right_velocity_dot + 2.0 * k3.right_velocity_dot + k4.right_velocity_dot) /
        6.0;
    next_state.left_wheel_position_m +=
        time_step_s *
        (k1.left_position_dot + 2.0 * k2.left_position_dot + 2.0 * k3.left_position_dot + k4.left_position_dot) /
        6.0;
    next_state.right_wheel_position_m +=
        time_step_s *
        (k1.right_position_dot + 2.0 * k2.right_position_dot + 2.0 * k3.right_position_dot + k4.right_position_dot) /
        6.0;
    return next_state;
}

RobotState DifferentialDriveDynamics::apply_derivatives(
    const RobotState& state,
    const Derivatives& derivatives,
    double scale,
    const EffectiveDriveParams& params) const {
    RobotState next_state = state;
    next_state.pose.x_m += derivatives.x_dot * scale;
    next_state.pose.y_m += derivatives.y_dot * scale;
    next_state.pose.theta_rad += derivatives.theta_dot * scale;
    next_state.left_wheel_velocity_mps += derivatives.left_velocity_dot * scale;
    next_state.right_wheel_velocity_mps += derivatives.right_velocity_dot * scale;
    next_state.left_wheel_position_m += derivatives.left_position_dot * scale;
    next_state.right_wheel_position_m += derivatives.right_position_dot * scale;
    next_state.linear_velocity_mps =
        0.5 * (next_state.left_wheel_velocity_mps + next_state.right_wheel_velocity_mps);
    next_state.angular_velocity_radps =
        (next_state.right_wheel_velocity_mps - next_state.left_wheel_velocity_mps) / params.track_width_m;
    return next_state;
}

double DifferentialDriveDynamics::compute_battery_voltage(
    const RobotInput& input,
    const EffectiveDriveParams& params) const {
    if (!params.enable_battery_sag) {
        return params.nominal_battery_voltage_v;
    }

    const double stall_current_per_motor_a =
        (base_drive_params_.motor.torque_constant_nm_per_a > 1e-6)
            ? (base_drive_params_.motor.stall_torque_nm / base_drive_params_.motor.torque_constant_nm_per_a)
            : 0.0;
    const double command_fraction =
        (std::abs(input.left_voltage_command_v) + std::abs(input.right_voltage_command_v)) /
        (2.0 * std::max(1e-6, params.nominal_battery_voltage_v));
    const double total_current_a =
        2.0 * static_cast<double>(base_drive_params_.motor.motor_count_per_side) *
        stall_current_per_motor_a *
        math::clamp(command_fraction, 0.0, 1.0);
    const double sagged_voltage_v =
        params.nominal_battery_voltage_v - params.battery_internal_resistance_ohm * total_current_a;
    return std::max(params.min_battery_voltage_v, sagged_voltage_v);
}

}  // namespace robot_sim
