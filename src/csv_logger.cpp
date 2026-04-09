/*
 * Purpose:
 *   Implements CSV writing for simulator and analysis outputs.
 *
 * Role In The Architecture:
 *   This translation unit is a pure reporting component. It contains no
 *   control, sensing, or physics logic; it only serializes already computed
 *   repository results.
 *
 * Key Functions:
 *   - CSVLogger::write_simulation_trace
 *   - CSVLogger::write_scenario_summaries
 *   - CSVLogger::write_monte_carlo_summary
 *   - CSVLogger::write_structural_screening
 *
 * Dependencies:
 *   - csv_logger.hpp
 *   - standard filesystem and stream support
 */

#include "csv_logger.hpp"

#include <fstream>
#include <stdexcept>
#include <utility>

namespace robot_sim {

CSVLogger::CSVLogger(std::filesystem::path output_directory)
    : output_directory_(std::move(output_directory)) {}

const std::filesystem::path& CSVLogger::output_directory() const noexcept {
    return output_directory_;
}

std::string CSVLogger::escape_csv(std::string value) {
    const bool needs_quotes =
        value.find(',') != std::string::npos ||
        value.find('"') != std::string::npos ||
        value.find('\n') != std::string::npos;

    // CSV quoting rule: embedded double quotes are escaped by doubling them.
    std::size_t search_position = 0;
    while ((search_position = value.find('"', search_position)) != std::string::npos) {
        value.insert(search_position, 1, '"');
        search_position += 2;
    }

    if (needs_quotes) {
        value.insert(value.begin(), '"');
        value.push_back('"');
    }

    return value;
}

void CSVLogger::ensure_output_directory() const {
    std::error_code error;
    std::filesystem::create_directories(output_directory_, error);
    if (error) {
        throw std::runtime_error("Failed to create output directory: " + output_directory_.string());
    }
}

std::filesystem::path CSVLogger::write_simulation_trace(
    const std::string& filename,
    const std::vector<SimulationTraceRow>& rows) const {
    ensure_output_directory();
    const auto path = output_directory_ / filename;
    std::ofstream output_stream(path);
    if (!output_stream) {
        throw std::runtime_error("Failed to open trace CSV for writing: " + path.string());
    }

    output_stream
        << "scenario_name,run_id,time_s,x_m,y_m,theta_rad,left_velocity_mps,right_velocity_mps,omega_radps,"
           "left_voltage_v,right_voltage_v,estimated_x_m,estimated_y_m,estimated_theta_rad,left_encoder_m,"
           "right_encoder_m,imu_heading_rad,gyro_z_radps,position_error_m,heading_error_rad\n";

    for (const auto& row : rows) {
        output_stream << escape_csv(row.scenario_name) << ','
                      << escape_csv(row.run_id) << ','
                      << row.time_s << ','
                      << row.x_m << ','
                      << row.y_m << ','
                      << row.theta_rad << ','
                      << row.left_velocity_mps << ','
                      << row.right_velocity_mps << ','
                      << row.omega_radps << ','
                      << row.left_voltage_v << ','
                      << row.right_voltage_v << ','
                      << row.estimated_x_m << ','
                      << row.estimated_y_m << ','
                      << row.estimated_theta_rad << ','
                      << row.left_encoder_m << ','
                      << row.right_encoder_m << ','
                      << row.imu_heading_rad << ','
                      << row.gyro_z_radps << ','
                      << row.position_error_m << ','
                      << row.heading_error_rad << '\n';
    }

    return path;
}

std::filesystem::path CSVLogger::write_scenario_summaries(
    const std::string& filename,
    const std::vector<ScenarioSummary>& summaries) const {
    ensure_output_directory();
    const auto path = output_directory_ / filename;
    std::ofstream output_stream(path);
    if (!output_stream) {
        throw std::runtime_error("Failed to open summary CSV for writing: " + path.string());
    }

    output_stream
        << "scenario_name,run_id,success,final_position_error_m,final_heading_error_rad,rms_path_error_m,"
           "max_path_error_m,overshoot_m,settling_time_s,duration_s,termination_reason,sample_count\n";

    for (const auto& summary : summaries) {
        output_stream << escape_csv(summary.scenario_name) << ','
                      << escape_csv(summary.run_id) << ','
                      << (summary.success ? 1 : 0) << ','
                      << summary.final_position_error_m << ','
                      << summary.final_heading_error_rad << ','
                      << summary.rms_path_error_m << ','
                      << summary.max_path_error_m << ','
                      << summary.overshoot_m << ','
                      << summary.settling_time_s << ','
                      << summary.duration_s << ','
                      << escape_csv(summary.termination_reason) << ','
                      << summary.sample_count << '\n';
    }

    return path;
}

std::filesystem::path CSVLogger::write_monte_carlo_summary(
    const std::string& filename,
    const MonteCarloSummary& summary) const {
    ensure_output_directory();
    const auto path = output_directory_ / filename;
    std::ofstream output_stream(path);
    if (!output_stream) {
        throw std::runtime_error("Failed to open Monte Carlo CSV for writing: " + path.string());
    }

    output_stream
        << "scenario_name,run_count,success_count,success_rate,mean_final_position_error_m,"
           "stddev_final_position_error_m,mean_final_heading_error_rad,stddev_final_heading_error_rad,"
           "mean_rms_path_error_m,stddev_rms_path_error_m,worst_case_position_error_m,"
           "worst_case_heading_error_rad,best_run_id,worst_run_id\n";
    output_stream << escape_csv(summary.scenario_name) << ','
                  << summary.run_count << ','
                  << summary.success_count << ','
                  << summary.success_rate << ','
                  << summary.mean_final_position_error_m << ','
                  << summary.stddev_final_position_error_m << ','
                  << summary.mean_final_heading_error_rad << ','
                  << summary.stddev_final_heading_error_rad << ','
                  << summary.mean_rms_path_error_m << ','
                  << summary.stddev_rms_path_error_m << ','
                  << summary.worst_case_position_error_m << ','
                  << summary.worst_case_heading_error_rad << ','
                  << escape_csv(summary.best_run_id) << ','
                  << escape_csv(summary.worst_run_id) << '\n';

    return path;
}

std::filesystem::path CSVLogger::write_structural_screening(
    const std::string& filename,
    const std::vector<StructuralAnalysisResult>& results) const {
    ensure_output_directory();
    const auto path = output_directory_ / filename;
    std::ofstream output_stream(path);
    if (!output_stream) {
        throw std::runtime_error("Failed to open structural CSV for writing: " + path.string());
    }

    output_stream
        << "member_name,load_case_name,max_reaction_n,max_shear_n,max_bending_moment_nm,"
           "max_bending_stress_pa,average_shear_stress_pa,safety_factor_indicator,"
           "likely_critical_region,warning\n";

    for (const auto& result : results) {
        output_stream << escape_csv(result.member_name) << ','
                      << escape_csv(result.load_case_name) << ','
                      << result.max_reaction_n << ','
                      << result.max_shear_n << ','
                      << result.max_bending_moment_nm << ','
                      << result.max_bending_stress_pa << ','
                      << result.average_shear_stress_pa << ','
                      << result.safety_factor_indicator << ','
                      << escape_csv(result.likely_critical_region) << ','
                      << escape_csv(result.warning) << '\n';
    }

    return path;
}

}  // namespace robot_sim
