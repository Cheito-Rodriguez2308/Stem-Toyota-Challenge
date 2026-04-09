/*
 * Purpose:
 *   Implements task-level robot controllers for the current simulation stack.
 *
 * Role In The Architecture:
 *   This translation unit belongs to the control layer. It transforms the
 *   estimated state and scenario goals into left/right voltage commands while
 *   relying on other modules for sensing and plant dynamics.
 *
 * Key Functions:
 *   - PointToPointController::compute_command
 *   - TurnInPlaceController::compute_command
 *   - make_controller
 *
 * Dependencies:
 *   - robot_controllers.hpp
 *   - math_utils.hpp for angle wrapping and clamping
 */

#include "robot_controllers.hpp"

#include "math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot_sim {
namespace {

/// Computes a stable controller time step [s] even on the first iteration.
double control_timestep(
    double time_s,
    double previous_time_s,
    bool has_previous_time,
    double fallback_timestep_s) {
    if (!has_previous_time) {
        return std::max(1e-3, fallback_timestep_s);
    }
    return std::max(1e-4, time_s - previous_time_s);
}

/// Returns the voltage magnitude limit currently available to the controller [V].
double available_voltage_limit(
    const ScenarioParams& scenario,
    const RobotState& true_state,
    const DriveParams& drive_params) {
    if (scenario.enable_battery_sag || drive_params.enable_battery_sag) {
        return std::max(1.0, true_state.battery_voltage_v);
    }
    return drive_params.nominal_battery_voltage_v;
}

/// Packages left/right voltage requests after saturation and slew-rate limiting.
RobotInput make_voltage_command(
    double left_voltage_v,
    double right_voltage_v,
    double voltage_limit_v,
    SlewRateLimiter& left_limiter,
    SlewRateLimiter& right_limiter,
    double time_step_s) {
    RobotInput input{};
    input.left_voltage_command_v = left_limiter.limit(
        math::clamp(left_voltage_v, -voltage_limit_v, voltage_limit_v),
        time_step_s);
    input.right_voltage_command_v = right_limiter.limit(
        math::clamp(right_voltage_v, -voltage_limit_v, voltage_limit_v),
        time_step_s);
    return input;
}

/// Converts a pose position into an Eigen 2D vector [m].
Eigen::Vector2d pose_vector(const Pose2D& pose) {
    return Eigen::Vector2d(pose.x_m, pose.y_m);
}

/// Chooses the turn controller for scenarios that should rotate in place.
bool use_turn_controller(const ScenarioParams& scenario) {
    return scenario.type == ScenarioType::NinetyDegreeTurn ||
           scenario.type == ScenarioType::InPlaceTurn ||
           scenario.controller_name == "turn_in_place";
}

}  // namespace

PointToPointController::PointToPointController(
    DriveParams drive_params,
    ControllerParams controller_params)
    : drive_params_(std::move(drive_params)),
      controller_params_(std::move(controller_params)),
      distance_pid_(controller_params_.distance_pid),
      heading_pid_(controller_params_.heading_pid),
      left_limiter_(controller_params_.max_voltage_step_per_s),
      right_limiter_(controller_params_.max_voltage_step_per_s),
      feedforward_(controller_params_.feedforward_kv, controller_params_.feedforward_ka) {}

void PointToPointController::reset(const ScenarioParams& scenario, const RobotState& initial_state) {
    Controller::reset(scenario, initial_state);
    scenario_ = scenario;
    distance_pid_.reset();
    heading_pid_.reset();
    left_limiter_.reset(0.0);
    right_limiter_.reset(0.0);
    previous_time_s_ = initial_state.sim_time_s;
    has_previous_time_ = false;
}

RobotInput PointToPointController::compute_command(
    double time_s,
    const RobotState& true_state,
    const EstimatedState& estimated_state,
    const SensorReadings& sensors) {
    (void)sensors;

    const double time_step_s =
        control_timestep(time_s, previous_time_s_, has_previous_time_, scenario_.time_step_s);
    previous_time_s_ = time_s;
    has_previous_time_ = true;

    const Eigen::Vector2d goal_position_m = scenario_.target_position_m;
    const Eigen::Vector2d current_position_m = pose_vector(estimated_state.pose);
    const Eigen::Vector2d position_error_vector_m = goal_position_m - current_position_m;
    const double distance_error_m = position_error_vector_m.norm();

    double forward_voltage_v = 0.0;
    double turn_voltage_v = 0.0;

    // Engineering Notes:
    //   This controller decomposes the task into:
    //     1. A distance loop for forward authority
    //     2. A heading loop for turn authority
    //   The heading target is the line-of-sight bearing until the robot is near
    //   the goal, then it switches to the requested final heading.
    // Assumptions:
    //   - The estimated pose is good enough for line-of-sight steering
    //   - A simple sum/difference mixing to left/right voltage is adequate
    // What Could Break In Reality:
    //   - Large latency or odometry drift
    //   - Wheel slip during aggressive turns
    //   - Actuator deadband or saturation not captured by this simple mixer
    if (distance_error_m <= controller_params_.goal_tolerance_m) {
        const double final_heading_error_rad =
            math::wrap_angle(scenario_.target_heading_rad - estimated_state.pose.theta_rad);
        turn_voltage_v = heading_pid_.update(final_heading_error_rad, time_step_s);
    } else {
        const double target_heading_rad =
            std::atan2(position_error_vector_m.y(), position_error_vector_m.x());
        const double heading_error_rad =
            math::wrap_angle(target_heading_rad - estimated_state.pose.theta_rad);
        const double heading_scale = std::max(0.0, std::cos(heading_error_rad));
        const double desired_speed_mps =
            controller_params_.max_linear_speed_mps *
            std::min(1.0, distance_error_m) *
            heading_scale;

        forward_voltage_v =
            feedforward_.calculate(desired_speed_mps, 0.0) +
            distance_pid_.update(distance_error_m, time_step_s) * heading_scale;
        turn_voltage_v = heading_pid_.update(heading_error_rad, time_step_s);
    }

    const double voltage_limit_v =
        available_voltage_limit(scenario_, true_state, drive_params_);
    return make_voltage_command(
        forward_voltage_v - turn_voltage_v,
        forward_voltage_v + turn_voltage_v,
        voltage_limit_v,
        left_limiter_,
        right_limiter_,
        time_step_s);
}

TurnInPlaceController::TurnInPlaceController(
    DriveParams drive_params,
    ControllerParams controller_params)
    : drive_params_(std::move(drive_params)),
      controller_params_(std::move(controller_params)),
      heading_pid_(controller_params_.heading_pid),
      left_limiter_(controller_params_.max_voltage_step_per_s),
      right_limiter_(controller_params_.max_voltage_step_per_s) {}

void TurnInPlaceController::reset(const ScenarioParams& scenario, const RobotState& initial_state) {
    Controller::reset(scenario, initial_state);
    scenario_ = scenario;
    heading_pid_.reset();
    left_limiter_.reset(0.0);
    right_limiter_.reset(0.0);
    previous_time_s_ = initial_state.sim_time_s;
    has_previous_time_ = false;
}

RobotInput TurnInPlaceController::compute_command(
    double time_s,
    const RobotState& true_state,
    const EstimatedState& estimated_state,
    const SensorReadings& sensors) {
    (void)sensors;

    const double time_step_s =
        control_timestep(time_s, previous_time_s_, has_previous_time_, scenario_.time_step_s);
    previous_time_s_ = time_s;
    has_previous_time_ = true;

    const double heading_error_rad =
        math::wrap_angle(scenario_.target_heading_rad - estimated_state.pose.theta_rad);
    const double turn_voltage_v = heading_pid_.update(heading_error_rad, time_step_s);
    const double voltage_limit_v =
        available_voltage_limit(scenario_, true_state, drive_params_);

    return make_voltage_command(
        -turn_voltage_v,
        turn_voltage_v,
        voltage_limit_v,
        left_limiter_,
        right_limiter_,
        time_step_s);
}

std::unique_ptr<Controller> make_controller(
    const ScenarioParams& scenario,
    const DriveParams& drive_params,
    const ControllerParams& controller_params) {
    if (use_turn_controller(scenario)) {
        return std::make_unique<TurnInPlaceController>(drive_params, controller_params);
    }
    return std::make_unique<PointToPointController>(drive_params, controller_params);
}

}  // namespace robot_sim
