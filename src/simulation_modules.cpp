/*
 * Purpose:
 *   Implements the lightweight default module behaviors used by the simulator
 *   pipeline.
 *
 * Role In The Architecture:
 *   This file provides baseline implementations for initialization, perfect
 *   sensing/estimation, null control, and goal-based stopping. More realistic
 *   models derive from the interfaces declared in simulation_modules.hpp.
 *
 * Key Functions:
 *   - DefaultStateInitializer::initialize
 *   - PerfectSensorModel::sample
 *   - GoalToleranceStopCondition::should_stop
 *
 * Dependencies:
 *   - simulation_modules.hpp
 *   - math_utils.hpp for goal-distance calculations
 */

#include "simulation_modules.hpp"

#include "math_utils.hpp"

#include <cmath>

namespace robot_sim {

void StateInitializer::reset(const ScenarioParams& scenario, const std::string& run_id) {
    (void)scenario;
    (void)run_id;
}

void SensorModel::reset(const ScenarioParams& scenario, const RobotState& initial_state) {
    (void)scenario;
    (void)initial_state;
}

void StateEstimator::reset(const ScenarioParams& scenario, const RobotState& initial_state) {
    (void)scenario;
    (void)initial_state;
}

void Controller::reset(const ScenarioParams& scenario, const RobotState& initial_state) {
    (void)scenario;
    (void)initial_state;
}

void DynamicsModel::reset(const ScenarioParams& scenario, const RobotState& initial_state) {
    (void)scenario;
    (void)initial_state;
}

void StopCondition::reset(const ScenarioParams& scenario, const RobotState& initial_state) {
    (void)scenario;
    (void)initial_state;
}

RobotState DefaultStateInitializer::initialize(const ScenarioParams& scenario) {
    RobotState initial_state{};
    initial_state.pose = scenario.initial_pose;
    initial_state.battery_voltage_v = scenario.battery_voltage_v;
    return initial_state;
}

SensorReadings PerfectSensorModel::sample(double time_s, const RobotState& state) {
    SensorReadings sensors{};
    sensors.timestamp_s = time_s;
    sensors.left_encoder_m = state.left_wheel_position_m;
    sensors.right_encoder_m = state.right_wheel_position_m;
    sensors.imu_heading_rad = state.pose.theta_rad;
    sensors.gyro_z_radps = state.angular_velocity_radps;
    return sensors;
}

EstimatedState PerfectStateEstimator::estimate(
    double time_s,
    const RobotState& true_state,
    const SensorReadings& sensors) {
    (void)time_s;
    (void)sensors;

    EstimatedState estimate{};
    estimate.pose = true_state.pose;
    estimate.linear_velocity_mps = true_state.linear_velocity_mps;
    estimate.angular_velocity_radps = true_state.angular_velocity_radps;
    return estimate;
}

RobotInput NullController::compute_command(
    double time_s,
    const RobotState& true_state,
    const EstimatedState& estimated_state,
    const SensorReadings& sensors) {
    (void)time_s;
    (void)true_state;
    (void)estimated_state;
    (void)sensors;
    return {};
}

void GoalToleranceStopCondition::reset(const ScenarioParams& scenario, const RobotState& initial_state) {
    (void)initial_state;
    scenario_ = scenario;
}

bool GoalToleranceStopCondition::should_stop(
    double time_s,
    const RobotState& true_state,
    const EstimatedState& estimated_state) const {
    (void)time_s;
    (void)estimated_state;

    const double position_error_m =
        math::euclidean_distance(true_state.pose, scenario_.target_position_m);
    const double heading_error_rad =
        math::wrap_angle(true_state.pose.theta_rad - scenario_.target_heading_rad);

    return scenario_.stop_on_goal &&
           position_error_m <= scenario_.stop_position_tolerance_m &&
           std::abs(heading_error_rad) <= scenario_.stop_heading_tolerance_rad;
}

}  // namespace robot_sim
