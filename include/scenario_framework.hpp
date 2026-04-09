#pragma once

/*
 * Purpose:
 *   Declares scenario factory helpers and the high-level runner that wires the
 *   modules together for executable examples.
 *
 * Role In The Architecture:
 *   This file belongs to the orchestration layer. It bridges configuration,
 *   controller selection, sensing, dynamics, simulator execution, and CSV
 *   logging into a convenient workflow for scenario studies.
 *
 * Key Classes:
 *   - ScenarioFactory
 *   - ScenarioRunner
 *
 * Dependencies:
 *   - config_loader.hpp
 *   - csv_logger.hpp
 *   - drivetrain_model.hpp
 *   - robot_controllers.hpp
 *   - sensor_simulator.hpp
 *   - simulator.hpp
 */

#include "config_loader.hpp"
#include "csv_logger.hpp"
#include "drivetrain_model.hpp"
#include "robot_controllers.hpp"
#include "sensor_simulator.hpp"
#include "simulator.hpp"

#include <filesystem>

namespace robot_sim {

/**
 * Provides reusable scenario presets for examples and regression-style runs.
 *
 * Layer:
 *   Scenario definition / orchestration.
 */
class ScenarioFactory {
public:
    /// Straight-line drive to a forward goal.
    static ScenarioParams make_straight_line();

    /// In-place 90-degree heading change.
    static ScenarioParams make_turn_in_place();

    /// Point-to-point autonomous move with nonzero final heading.
    static ScenarioParams make_point_to_point();

    /// Standard three-scenario example suite.
    static std::vector<ScenarioParams> standard_suite();
};

/**
 * High-level helper that executes scenarios and writes CSV outputs.
 *
 * Layer:
 *   Orchestration / examples.
 *
 * Usage:
 *   Construct once with repository parameter groups, then call run_scenario()
 *   or run_batch() from an executable or higher-level study runner.
 */
class ScenarioRunner {
public:
    ScenarioRunner(
        DriveParams drive_params,
        SensorParams sensor_params,
        ControllerParams controller_params,
        std::filesystem::path output_directory = "output",
        std::uint32_t seed = 1U);

    /// Runs one scenario and writes per-run CSV outputs.
    SimulationResult run_scenario(
        const ScenarioParams& scenario,
        const std::string& run_id = "run_000");

    /// Runs a batch of scenarios and writes per-run plus aggregate summary CSVs.
    std::vector<SimulationResult> run_batch(
        const std::vector<ScenarioParams>& scenarios,
        const std::string& batch_prefix = "batch");

private:
    DriveParams drive_params_{};
    SensorParams sensor_params_{};
    ControllerParams controller_params_{};
    CSVLogger logger_;
    std::uint32_t seed_{1U};
};

}  // namespace robot_sim
