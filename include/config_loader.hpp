#pragma once

/*
 * Purpose:
 *   Declares JSON parsing helpers that convert configuration files into the
 *   strongly typed repository data structures from core_types.hpp.
 *
 * Role In The Architecture:
 *   This header isolates configuration I/O from simulation, sensing, control,
 *   and reporting logic. Other modules consume typed parameters instead of raw
 *   JSON trees.
 *
 * Key Functions:
 *   - ConfigLoader::load_json_file
 *   - ConfigLoader::load_bundle
 *   - ConfigLoader::load_bundle_from_file
 *
 * Dependencies:
 *   - core_types.hpp for output structures
 *   - nlohmann/json for parsing
 *   - std::filesystem for file paths
 */

#include "core_types.hpp"

#include <filesystem>

#include <nlohmann/json.hpp>

namespace robot_sim {

/**
 * Static JSON configuration loader.
 *
 * Layer:
 *   Configuration / file I/O boundary.
 *
 * Usage:
 *   Use load_bundle_from_file() for top-level program entry points, or the
 *   narrower parsing helpers when only one parameter block is needed.
 */
class ConfigLoader {
public:
    /// Reads and parses a JSON file from disk.
    static nlohmann::json load_json_file(const std::filesystem::path& path);

    /// Parses drivetrain parameters expressed in SI units.
    static DriveParams load_drive_params(const nlohmann::json& json);

    /// Parses sensor parameters and channel error models.
    static SensorParams load_sensor_params(const nlohmann::json& json);

    /// Parses controller gains and output limits.
    static ControllerParams load_controller_params(const nlohmann::json& json);

    /// Parses one scenario definition.
    static ScenarioParams load_scenario_params(const nlohmann::json& json);

    /// Parses Monte Carlo configuration ranges.
    static MonteCarloConfig load_monte_carlo_config(const nlohmann::json& json);

    /// Parses one simplified structural member definition.
    static StructuralMember load_structural_member(const nlohmann::json& json);

    /// Parses one simplified structural load case definition.
    static StructuralLoadCase load_structural_load_case(const nlohmann::json& json);

    /// Parses an entire repository configuration bundle.
    static SimulationConfigBundle load_bundle(const nlohmann::json& json);

    /// Loads a complete bundle directly from a JSON file on disk.
    static SimulationConfigBundle load_bundle_from_file(const std::filesystem::path& path);
};

}  // namespace robot_sim
