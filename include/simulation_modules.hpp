#pragma once

/*
 * Purpose:
 *   Declares the abstract module interfaces that make the simulator pipeline
 *   pluggable.
 *
 * Role In The Architecture:
 *   This header separates the simulation loop from specific implementations of
 *   initialization, sensing, estimation, control, dynamics, and stop logic.
 *   That separation makes the code easier to extend without creating a single
 *   monolithic simulator class.
 *
 * Key Types:
 *   - StateInitializer
 *   - SensorModel
 *   - StateEstimator
 *   - Controller
 *   - DynamicsModel
 *   - StopCondition
 *   - SimulationPipeline
 *
 * Dependencies:
 *   - core_types.hpp for shared state and parameter structs
 */

#include "core_types.hpp"

#include <string>

namespace robot_sim {

/**
 * Produces the initial RobotState for a scenario.
 *
 * Layer:
 *   Simulation orchestration.
 */
class StateInitializer {
public:
    virtual ~StateInitializer() = default;

    /// Optional reset hook called before initialize() for a new run.
    virtual void reset(const ScenarioParams& scenario, const std::string& run_id);

    /// Produces the initial true robot state for the given scenario.
    virtual RobotState initialize(const ScenarioParams& scenario) = 0;
};

/**
 * Converts the true robot state into simulated sensor outputs.
 *
 * Layer:
 *   Sensor modeling.
 */
class SensorModel {
public:
    virtual ~SensorModel() = default;

    /// Optional reset hook called before the first sample of a run.
    virtual void reset(const ScenarioParams& scenario, const RobotState& initial_state);

    /// Samples the sensor suite at time_s [s] from the current true state.
    virtual SensorReadings sample(double time_s, const RobotState& state) = 0;
};

/**
 * Estimates robot state from sensor outputs.
 *
 * Layer:
 *   Estimation.
 */
class StateEstimator {
public:
    virtual ~StateEstimator() = default;

    /// Optional reset hook called before the first estimate of a run.
    virtual void reset(const ScenarioParams& scenario, const RobotState& initial_state);

    /// Produces a state estimate at time_s [s] using the current sensor sample.
    virtual EstimatedState estimate(
        double time_s,
        const RobotState& true_state,
        const SensorReadings& sensors) = 0;
};

/**
 * Computes the next plant input from the estimated state and sensor readings.
 *
 * Layer:
 *   Control.
 */
class Controller {
public:
    virtual ~Controller() = default;

    /// Optional reset hook called before the first control update of a run.
    virtual void reset(const ScenarioParams& scenario, const RobotState& initial_state);

    /// Computes the control input to apply at the current simulation step.
    virtual RobotInput compute_command(
        double time_s,
        const RobotState& true_state,
        const EstimatedState& estimated_state,
        const SensorReadings& sensors) = 0;
};

/**
 * Advances the ground-truth robot state by one fixed time step.
 *
 * Layer:
 *   Simulation / plant dynamics.
 */
class DynamicsModel {
public:
    virtual ~DynamicsModel() = default;

    /// Optional reset hook called before the first dynamics update of a run.
    virtual void reset(const ScenarioParams& scenario, const RobotState& initial_state);

    /// Steps the plant from state(t) to state(t + dt).
    virtual RobotState step(
        double time_s,
        double time_step_s,
        const RobotState& state,
        const RobotInput& input) = 0;
};

/**
 * Decides whether a simulation should stop early.
 *
 * Layer:
 *   Simulation orchestration.
 */
class StopCondition {
public:
    virtual ~StopCondition() = default;

    /// Optional reset hook called before the run begins.
    virtual void reset(const ScenarioParams& scenario, const RobotState& initial_state);

    /// Returns true when the run should terminate.
    virtual bool should_stop(
        double time_s,
        const RobotState& true_state,
        const EstimatedState& estimated_state) const = 0;
};

/**
 * Collection of module pointers used by Simulator::run().
 *
 * Usage:
 *   The pipeline is non-owning. The caller remains responsible for the module
 *   lifetimes.
 */
struct SimulationPipeline {
    StateInitializer* initializer{nullptr}; ///< Produces the initial true state.
    SensorModel* sensor_model{nullptr};     ///< Generates simulated measurements.
    StateEstimator* estimator{nullptr};     ///< Produces estimated state.
    Controller* controller{nullptr};        ///< Produces robot input commands.
    DynamicsModel* dynamics{nullptr};       ///< Advances the true state.
    StopCondition* stop_condition{nullptr}; ///< Optional early-termination logic.
};

/**
 * Default initializer that maps ScenarioParams::initial_pose into RobotState.
 */
class DefaultStateInitializer final : public StateInitializer {
public:
    RobotState initialize(const ScenarioParams& scenario) override;
};

/**
 * Sensor model that reports perfect noiseless measurements.
 *
 * Usage:
 *   Useful as a baseline or for debugging estimator-independent behavior.
 */
class PerfectSensorModel final : public SensorModel {
public:
    SensorReadings sample(double time_s, const RobotState& state) override;
};

/**
 * State estimator that exposes the true state directly.
 *
 * Usage:
 *   Useful for debugging controllers or dynamics independently of sensor error.
 */
class PerfectStateEstimator final : public StateEstimator {
public:
    EstimatedState estimate(
        double time_s,
        const RobotState& true_state,
        const SensorReadings& sensors) override;
};

/**
 * Controller that always commands zero input.
 *
 * Usage:
 *   Useful for open-loop initialization checks and infrastructure testing.
 */
class NullController final : public Controller {
public:
    RobotInput compute_command(
        double time_s,
        const RobotState& true_state,
        const EstimatedState& estimated_state,
        const SensorReadings& sensors) override;
};

/**
 * Stop condition based on goal position and heading tolerances.
 *
 * Assumptions:
 *   Goal completion is judged geometrically from the true state and scenario
 *   tolerances.
 */
class GoalToleranceStopCondition final : public StopCondition {
public:
    void reset(const ScenarioParams& scenario, const RobotState& initial_state) override;
    bool should_stop(
        double time_s,
        const RobotState& true_state,
        const EstimatedState& estimated_state) const override;

private:
    ScenarioParams scenario_{}; ///< Stored scenario tolerances and goal definition.
};

}  // namespace robot_sim
