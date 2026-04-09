/*
 * Purpose:
 *   Implements the fixed-step simulation loop and summary generation.
 *
 * Role In The Architecture:
 *   This file coordinates the top-level robotics workflow for one scenario:
 *   initialize state, sample sensors, estimate state, compute control, advance
 *   dynamics, log rows, and summarize the outcome.
 *
 * Key Functions:
 *   - Simulator::run
 *   - Simulator::summarize
 *
 * Dependencies:
 *   - simulator.hpp
 *   - math_utils.hpp for distance and heading-error calculations
 */

#include "simulator.hpp"

#include "math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace robot_sim {
namespace {

/// Computes planar distance from the current true pose to the scenario target [m].
double distance_to_target(const RobotState& state, const ScenarioParams& scenario) {
    return math::euclidean_distance(state.pose, scenario.target_position_m);
}

}  // namespace

Simulator::Simulator(double default_time_step_s)
    : default_time_step_s_(default_time_step_s) {
    if (default_time_step_s_ <= 0.0) {
        throw std::invalid_argument("Simulator time step must be positive.");
    }
}

double Simulator::default_time_step_s() const noexcept {
    return default_time_step_s_;
}

SimulationResult Simulator::run(
    const ScenarioParams& scenario,
    SimulationPipeline& pipeline,
    const std::string& run_id) const {
    if (pipeline.initializer == nullptr ||
        pipeline.sensor_model == nullptr ||
        pipeline.estimator == nullptr ||
        pipeline.controller == nullptr ||
        pipeline.dynamics == nullptr) {
        throw std::invalid_argument(
            "SimulationPipeline requires initializer, sensor_model, estimator, controller, and dynamics.");
    }

    pipeline.initializer->reset(scenario, run_id);
    const RobotState initial_state = pipeline.initializer->initialize(scenario);
    pipeline.sensor_model->reset(scenario, initial_state);
    pipeline.estimator->reset(scenario, initial_state);
    pipeline.controller->reset(scenario, initial_state);
    pipeline.dynamics->reset(scenario, initial_state);
    if (pipeline.stop_condition != nullptr) {
        pipeline.stop_condition->reset(scenario, initial_state);
    }

    SimulationHooks hooks{};
    hooks.initialize_state = [initial_state](const ScenarioParams&) {
        return initial_state;
    };
    hooks.controller = [&pipeline](
                           double time_s,
                           const RobotState& true_state,
                           const EstimatedState& estimated_state,
                           const SensorReadings& sensors) {
        return pipeline.controller->compute_command(time_s, true_state, estimated_state, sensors);
    };
    hooks.sensor_model = [&pipeline](double time_s, const RobotState& state) {
        return pipeline.sensor_model->sample(time_s, state);
    };
    hooks.estimator = [&pipeline](
                          double time_s,
                          const RobotState& true_state,
                          const SensorReadings& sensors) {
        return pipeline.estimator->estimate(time_s, true_state, sensors);
    };
    hooks.dynamics_step = [&pipeline](
                              double time_s,
                              double time_step_s,
                              const RobotState& state,
                              const RobotInput& input) {
        return pipeline.dynamics->step(time_s, time_step_s, state, input);
    };
    if (pipeline.stop_condition != nullptr) {
        hooks.stop_condition = [&pipeline](
                                   double time_s,
                                   const RobotState& true_state,
                                   const EstimatedState& estimated_state) {
            return pipeline.stop_condition->should_stop(time_s, true_state, estimated_state);
        };
    }

    return run(scenario, hooks, run_id);
}

SimulationResult Simulator::run(
    const ScenarioParams& scenario,
    const SimulationHooks& hooks,
    const std::string& run_id) const {
    if (!hooks.initialize_state ||
        !hooks.controller ||
        !hooks.sensor_model ||
        !hooks.estimator ||
        !hooks.dynamics_step) {
        throw std::invalid_argument(
            "SimulationHooks must provide initialize_state, controller, sensor_model, estimator, and dynamics_step.");
    }

    const double time_step_s =
        scenario.time_step_s > 0.0 ? scenario.time_step_s : default_time_step_s_;
    RobotState state = hooks.initialize_state(scenario);
    state.sim_time_s = 0.0;

    std::vector<SimulationTraceRow> trace;
    trace.reserve(static_cast<std::size_t>(std::ceil(scenario.duration_s / time_step_s)) + 1U);
    std::string termination_reason = "duration_elapsed";

    for (double time_s = 0.0; time_s <= scenario.duration_s + 1e-9; time_s += time_step_s) {
        state.sim_time_s = time_s;
        const SensorReadings sensors = hooks.sensor_model(time_s, state);
        const EstimatedState estimate = hooks.estimator(time_s, state, sensors);
        const RobotInput input = hooks.controller(time_s, state, estimate, sensors);

        SimulationTraceRow row{};
        row.scenario_name = scenario.name;
        row.run_id = run_id;
        row.time_s = time_s;
        row.x_m = state.pose.x_m;
        row.y_m = state.pose.y_m;
        row.theta_rad = state.pose.theta_rad;
        row.left_velocity_mps = state.left_wheel_velocity_mps;
        row.right_velocity_mps = state.right_wheel_velocity_mps;
        row.omega_radps = state.angular_velocity_radps;
        row.left_voltage_v = input.left_voltage_command_v;
        row.right_voltage_v = input.right_voltage_command_v;
        row.estimated_x_m = estimate.pose.x_m;
        row.estimated_y_m = estimate.pose.y_m;
        row.estimated_theta_rad = estimate.pose.theta_rad;
        row.left_encoder_m = sensors.left_encoder_m;
        row.right_encoder_m = sensors.right_encoder_m;
        row.imu_heading_rad = sensors.imu_heading_rad;
        row.gyro_z_radps = sensors.gyro_z_radps;
        row.position_error_m = distance_to_target(state, scenario);
        row.heading_error_rad = math::wrap_angle(state.pose.theta_rad - scenario.target_heading_rad);
        trace.push_back(row);

        const bool stop_requested = hooks.stop_condition
            ? hooks.stop_condition(time_s, state, estimate)
            : (scenario.stop_on_goal &&
               row.position_error_m <= scenario.stop_position_tolerance_m &&
               std::abs(row.heading_error_rad) <= scenario.stop_heading_tolerance_rad);
        if (stop_requested) {
            termination_reason = hooks.stop_condition ? "custom_stop_condition" : "goal_tolerance_met";
            break;
        }

        state = hooks.dynamics_step(time_s, time_step_s, state, input);
    }

    SimulationResult result{};
    if (trace.empty()) {
        termination_reason = "no_samples";
    }
    result.summary = summarize(scenario, run_id, termination_reason, trace);
    result.trace = std::move(trace);
    return result;
}

ScenarioSummary Simulator::summarize(
    const ScenarioParams& scenario,
    const std::string& run_id,
    const std::string& termination_reason,
    const std::vector<SimulationTraceRow>& trace) {
    ScenarioSummary summary{};
    summary.scenario_name = scenario.name;
    summary.run_id = run_id;
    summary.termination_reason = termination_reason;
    summary.sample_count = trace.size();

    if (trace.empty()) {
        return summary;
    }

    const auto& final_row = trace.back();
    summary.final_position_error_m = final_row.position_error_m;
    summary.final_heading_error_rad = final_row.heading_error_rad;
    summary.duration_s = final_row.time_s;
    summary.success =
        final_row.position_error_m <= scenario.stop_position_tolerance_m &&
        std::abs(final_row.heading_error_rad) <= scenario.stop_heading_tolerance_rad;

    std::vector<double> position_errors_m;
    position_errors_m.reserve(trace.size());
    double maximum_error_m = 0.0;
    double overshoot_m = 0.0;
    bool has_settled = false;

    for (const auto& row : trace) {
        position_errors_m.push_back(row.position_error_m);
        maximum_error_m = std::max(maximum_error_m, row.position_error_m);
        overshoot_m = std::max(overshoot_m, row.position_error_m - summary.final_position_error_m);

        if (!has_settled &&
            row.position_error_m <= scenario.stop_position_tolerance_m &&
            std::abs(row.heading_error_rad) <= scenario.stop_heading_tolerance_rad) {
            summary.settling_time_s = row.time_s;
            has_settled = true;
        }
    }

    summary.rms_path_error_m = math::rms(position_errors_m);
    summary.max_path_error_m = maximum_error_m;
    summary.overshoot_m = overshoot_m;
    return summary;
}

}  // namespace robot_sim
