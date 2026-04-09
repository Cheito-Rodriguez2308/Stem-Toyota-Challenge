#pragma once

/*
 * Purpose:
 *   Declares the fixed-step simulation loop that orchestrates initialization,
 *   sensing, estimation, control, dynamics, logging rows, and summary metrics.
 *
 * Role In The Architecture:
 *   Simulator is the execution shell of the robotics stack. It does not own
 *   the specific physics or control implementations; instead it coordinates
 *   pluggable modules through interfaces or callback hooks.
 *
 * Key Types:
 *   - SimulationHooks
 *   - Simulator
 *
 * Dependencies:
 *   - core_types.hpp for shared data structures
 *   - simulation_modules.hpp for interface-based pipelines
 */

#include "core_types.hpp"
#include "simulation_modules.hpp"

#include <functional>
#include <string>

namespace robot_sim {

/**
 * Callback-based simulator plumbing used when concrete module objects are not
 * convenient.
 *
 * Usage:
 *   Each callback is optional only where explicitly documented. Most simulator
 *   execution requires initialize_state, controller, sensor_model, estimator,
 *   and dynamics_step.
 */
struct SimulationHooks {
    std::function<RobotState(const ScenarioParams&)> initialize_state;
    std::function<RobotInput(double, const RobotState&, const EstimatedState&, const SensorReadings&)> controller;
    std::function<SensorReadings(double, const RobotState&)> sensor_model;
    std::function<EstimatedState(double, const RobotState&, const SensorReadings&)> estimator;
    std::function<RobotState(double, double, const RobotState&, const RobotInput&)> dynamics_step;
    std::function<bool(double, const RobotState&, const EstimatedState&)> stop_condition;
};

/**
 * Fixed-step simulation loop for one scenario run.
 *
 * Layer:
 *   Simulation orchestration.
 *
 * Assumptions:
 *   - The simulation advances with a constant time step.
 *   - Modules are deterministic except where they intentionally use seeded RNG.
 *
 * Not Modeled:
 *   Event-driven multi-rate scheduling, asynchronous sensor buses, or real-time
 *   wall-clock execution.
 */
class Simulator {
public:
    /// Constructs the simulator with a default fixed time step [s].
    explicit Simulator(double default_time_step_s = 0.01);

    /// Returns the default integration step [s].
    double default_time_step_s() const noexcept;

    /**
     * Runs a scenario using interface-based pipeline modules.
     *
     * Input:
     *   - scenario: scenario definition and time-step settings.
     *   - pipeline: non-owning module pointers for initialization, sensing,
     *     estimation, control, dynamics, and optional stopping.
     *   - run_id: caller-supplied identifier used in logs and summaries.
     *
     * Output:
     *   SimulationResult containing trace rows and summary metrics.
     */
    SimulationResult run(
        const ScenarioParams& scenario,
        SimulationPipeline& pipeline,
        const std::string& run_id = "run_000") const;

    /**
     * Runs a scenario using callback hooks instead of module objects.
     */
    SimulationResult run(
        const ScenarioParams& scenario,
        const SimulationHooks& hooks,
        const std::string& run_id = "run_000") const;

private:
    double default_time_step_s_{0.01}; ///< Default fallback time step [s].

    /// Builds run-level summary metrics from a completed trace.
    static ScenarioSummary summarize(
        const ScenarioParams& scenario,
        const std::string& run_id,
        const std::string& termination_reason,
        const std::vector<SimulationTraceRow>& trace);
};

}  // namespace robot_sim
