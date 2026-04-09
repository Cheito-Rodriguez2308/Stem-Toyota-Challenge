#pragma once

/*
 * Purpose:
 *   Declares CSV export helpers for simulation traces, summaries, and later
 *   study outputs.
 *
 * Role In The Architecture:
 *   This file sits at the reporting boundary. Simulation and analysis modules
 *   produce typed results; CSVLogger serializes them into portable flat files.
 *
 * Key Functions:
 *   - write_simulation_trace
 *   - write_scenario_summaries
 *   - write_monte_carlo_summary
 *   - write_structural_screening
 *
 * Dependencies:
 *   - core_types.hpp for exported record types
 *   - std::filesystem for output paths
 */

#include "core_types.hpp"

#include <filesystem>
#include <vector>

namespace robot_sim {

/**
 * CSV file writer for repository outputs.
 *
 * Layer:
 *   Logging / reporting / file I/O.
 *
 * Usage:
 *   Construct once with an output directory, then call the appropriate write
 *   method for the result type being exported.
 */
class CSVLogger {
public:
    /// Constructs a logger rooted at the given output directory.
    explicit CSVLogger(std::filesystem::path output_directory);

    /// Returns the base directory where CSV files are written.
    const std::filesystem::path& output_directory() const noexcept;

    /// Writes a time-history simulation trace and returns the resulting path.
    std::filesystem::path write_simulation_trace(
        const std::string& filename,
        const std::vector<SimulationTraceRow>& rows) const;

    /// Writes one or more scenario summary rows and returns the resulting path.
    std::filesystem::path write_scenario_summaries(
        const std::string& filename,
        const std::vector<ScenarioSummary>& summaries) const;

    /// Writes one Monte Carlo summary row and returns the resulting path.
    std::filesystem::path write_monte_carlo_summary(
        const std::string& filename,
        const MonteCarloSummary& summary) const;

    /**
     * Writes simplified structural screening results.
     *
     * Limitation:
     *   These exports are screening outputs, not FEM-equivalent stress results.
     */
    std::filesystem::path write_structural_screening(
        const std::string& filename,
        const std::vector<StructuralAnalysisResult>& results) const;

private:
    std::filesystem::path output_directory_; ///< Root directory for all generated CSV files.

    /// Escapes commas, quotes, and newlines so string fields remain valid CSV.
    static std::string escape_csv(std::string value);

    /// Creates the output directory if it does not already exist.
    void ensure_output_directory() const;
};

}  // namespace robot_sim
