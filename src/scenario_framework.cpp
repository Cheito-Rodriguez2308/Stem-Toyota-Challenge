/*
 * Purpose:
 *   Implements scenario presets and the high-level scenario runner.
 *
 * Role In The Architecture:
 *   This translation unit coordinates concrete module construction for example
 *   runs without pushing that wiring logic into main() or into the simulator
 *   itself.
 *
 * Key Functions:
 *   - ScenarioFactory::standard_suite
 *   - ScenarioRunner::run_scenario
 *   - ScenarioRunner::run_batch
 *
 * Dependencies:
 *   - scenario_framework.hpp
 *   - math_utils.hpp for pi
 */

#include "scenario_framework.hpp"

#include "math_utils.hpp"

#include <cctype>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string_view>
#include <utility>

namespace robot_sim {
namespace {

/// Derives a stable per-run seed from a text identifier and a base seed.
std::uint32_t fnv1a_seed(std::string_view text, std::uint32_t basis) {
    std::uint32_t hash = 2166136261u ^ basis;
    for (const char character : text) {
        hash ^= static_cast<std::uint8_t>(character);
        hash *= 16777619u;
    }
    return hash;
}

/// Converts a scenario or run label into a filesystem-friendly file stem.
std::string sanitize_filename(std::string value) {
    for (char& character : value) {
        if (!std::isalnum(static_cast<unsigned char>(character)) &&
            character != '-' &&
            character != '_') {
            character = '_';
        }
    }
    return value;
}

/// Builds a simple numbered run identifier from a batch prefix.
std::string numbered_run_id(const std::string& prefix, std::size_t index) {
    std::ostringstream stream;
    stream << prefix << '_' << index;
    return sanitize_filename(stream.str());
}

}  // namespace

ScenarioParams ScenarioFactory::make_straight_line() {
    ScenarioParams scenario{};
    scenario.name = "straight_line_drive";
    scenario.description = "Drive 2 meters straight from rest.";
    scenario.type = ScenarioType::StraightLine;
    scenario.target_position_m = Eigen::Vector2d(2.0, 0.0);
    scenario.target_heading_rad = 0.0;
    scenario.duration_s = 5.0;
    scenario.time_step_s = 0.01;
    scenario.integrator = IntegratorType::RK4;
    scenario.controller_name = "point_to_point";
    return scenario;
}

ScenarioParams ScenarioFactory::make_turn_in_place() {
    ScenarioParams scenario{};
    scenario.name = "turn_in_place_90deg";
    scenario.description = "Rotate 90 degrees in place.";
    scenario.type = ScenarioType::InPlaceTurn;
    scenario.target_position_m = Eigen::Vector2d(0.0, 0.0);
    scenario.target_heading_rad = 0.5 * math::kPi;
    scenario.duration_s = 3.0;
    scenario.time_step_s = 0.01;
    scenario.integrator = IntegratorType::Euler;
    scenario.controller_name = "turn_in_place";
    return scenario;
}

ScenarioParams ScenarioFactory::make_point_to_point() {
    ScenarioParams scenario{};
    scenario.name = "point_to_point_autonomous";
    scenario.description = "Drive to an offset waypoint and settle on heading.";
    scenario.type = ScenarioType::PointToPoint;
    scenario.target_position_m = Eigen::Vector2d(1.8, 1.1);
    scenario.target_heading_rad = 0.35;
    scenario.duration_s = 6.0;
    scenario.time_step_s = 0.01;
    scenario.integrator = IntegratorType::RK4;
    scenario.controller_name = "point_to_point";
    return scenario;
}

std::vector<ScenarioParams> ScenarioFactory::standard_suite() {
    return {
        make_straight_line(),
        make_turn_in_place(),
        make_point_to_point()
    };
}

ScenarioRunner::ScenarioRunner(
    DriveParams drive_params,
    SensorParams sensor_params,
    ControllerParams controller_params,
    std::filesystem::path output_directory,
    std::uint32_t seed)
    : drive_params_(std::move(drive_params)),
      sensor_params_(std::move(sensor_params)),
      controller_params_(std::move(controller_params)),
      logger_(std::move(output_directory)),
      seed_(seed) {}

SimulationResult ScenarioRunner::run_scenario(
    const ScenarioParams& scenario,
    const std::string& run_id) {
    DefaultStateInitializer initializer;
    DifferentialDriveDynamics dynamics(drive_params_);
    DifferentialDriveSensorSimulator sensor_model(sensor_params_, fnv1a_seed(run_id, seed_));
    WheelOdometryEstimator odometry_estimator(drive_params_.track_width_m);
    PerfectStateEstimator perfect_estimator;
    GoalToleranceStopCondition stop_condition;
    std::unique_ptr<Controller> controller =
        make_controller(scenario, drive_params_, controller_params_);

    StateEstimator* estimator = sensor_params_.enable_odometry_estimator
        ? static_cast<StateEstimator*>(&odometry_estimator)
        : static_cast<StateEstimator*>(&perfect_estimator);

    SimulationPipeline pipeline{};
    pipeline.initializer = &initializer;
    pipeline.sensor_model = &sensor_model;
    pipeline.estimator = estimator;
    pipeline.controller = controller.get();
    pipeline.dynamics = &dynamics;
    pipeline.stop_condition = scenario.stop_on_goal ? &stop_condition : nullptr;

    Simulator simulator(scenario.time_step_s);
    SimulationResult result = simulator.run(scenario, pipeline, run_id);

    const std::string base_name = sanitize_filename(scenario.name + "_" + run_id);
    logger_.write_simulation_trace(base_name + "_trace.csv", result.trace);
    logger_.write_scenario_summaries(base_name + "_summary.csv", {result.summary});
    return result;
}

std::vector<SimulationResult> ScenarioRunner::run_batch(
    const std::vector<ScenarioParams>& scenarios,
    const std::string& batch_prefix) {
    std::vector<SimulationResult> results;
    std::vector<ScenarioSummary> summaries;
    results.reserve(scenarios.size());
    summaries.reserve(scenarios.size());

    for (std::size_t index = 0; index < scenarios.size(); ++index) {
        const std::string run_id = numbered_run_id(batch_prefix, index);
        SimulationResult result = run_scenario(scenarios[index], run_id);
        summaries.push_back(result.summary);
        results.push_back(std::move(result));
    }

    logger_.write_scenario_summaries(
        sanitize_filename(batch_prefix) + "_scenario_summaries.csv",
        summaries);
    return results;
}

}  // namespace robot_sim
