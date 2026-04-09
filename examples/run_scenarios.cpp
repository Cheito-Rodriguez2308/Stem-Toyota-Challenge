/*
 * Purpose:
 *   Minimal executable example that loads configuration, runs the configured
 *   scenarios, and writes CSV output.
 *
 * Role In The Architecture:
 *   This file is the simplest end-to-end entry point for the repository. It
 *   demonstrates how configuration loading, scenario orchestration, simulation,
 *   and reporting fit together.
 *
 * Key Steps:
 *   - Load SimulationConfigBundle from JSON
 *   - Choose scenario list
 *   - Run ScenarioRunner
 *   - Print summary metrics to stdout
 *
 * Dependencies:
 *   - scenario_framework.hpp
 *   - standard filesystem and stream support
 */

#include "scenario_framework.hpp"

#include <exception>
#include <filesystem>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    namespace fs = std::filesystem;
    using namespace robot_sim;

    try {
        const fs::path config_path = argc > 1
            ? fs::path(argv[1])
            : fs::path("data") / "sample_competition_robot.json";

        const SimulationConfigBundle bundle =
            ConfigLoader::load_bundle_from_file(config_path);
        const std::vector<ScenarioParams> scenarios =
            bundle.scenarios.empty() ? ScenarioFactory::standard_suite() : bundle.scenarios;

        ScenarioRunner runner(
            bundle.drive,
            bundle.sensors,
            bundle.controllers,
            "output",
            bundle.monte_carlo.seed);
        const auto results = runner.run_batch(scenarios, "phase3_examples");

        std::cout << "Ran " << results.size() << " scenarios from "
                  << config_path.string() << '\n';
        for (const auto& result : results) {
            const auto& summary = result.summary;
            std::cout << summary.scenario_name
                      << " | success=" << (summary.success ? "true" : "false")
                      << " | final_pos_err=" << summary.final_position_error_m
                      << " | final_heading_err=" << summary.final_heading_error_rad
                      << " | termination=" << summary.termination_reason
                      << '\n';
        }
    } catch (const std::exception& exception) {
        std::cerr << "Scenario run failed: " << exception.what() << '\n';
        return 1;
    }

    return 0;
}
