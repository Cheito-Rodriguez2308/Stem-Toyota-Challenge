#pragma once

/*
 * Purpose:
 *   Declares the differential-drive sensor simulation and the corresponding
 *   wheel-odometry estimator.
 *
 * Role In The Architecture:
 *   This file sits between the true plant state and the controller. It makes
 *   the separation between truth, measurements, and estimated state explicit.
 *
 * Key Classes:
 *   - DifferentialDriveSensorSimulator
 *   - WheelOdometryEstimator
 *
 * Dependencies:
 *   - random_utils.hpp for reproducible noise and fault injection
 *   - simulation_modules.hpp for SensorModel and StateEstimator interfaces
 */

#include "random_utils.hpp"
#include "simulation_modules.hpp"

#include <deque>

namespace robot_sim {

/**
 * Simulates encoder and IMU/gyro outputs for a differential-drive robot.
 *
 * Layer:
 *   Sensor modeling.
 *
 * Assumptions:
 *   - Each sensor channel uses an independent error model.
 *   - Delay is implemented by replaying buffered truth samples.
 *   - Drift is approximated as linearly accumulated bias.
 *
 * Not Modeled:
 *   - Temperature effects
 *   - Cross-axis IMU coupling
 *   - Vibration-sensitive bias changes
 *   - Bus scheduling and asynchronous packet timing
 *
 * Usage:
 *   Construct with SensorParams and a seed, call reset() at the start of a
 *   scenario, then call sample() once per simulator step.
 */
class DifferentialDriveSensorSimulator final : public SensorModel {
public:
    explicit DifferentialDriveSensorSimulator(SensorParams sensor_params, std::uint32_t seed = 1U);

    void reset(const ScenarioParams& scenario, const RobotState& initial_state) override;

    /**
     * Produces a sensor sample at the current simulator time.
     *
     * Input:
     *   - time_s: sample time [s].
     *   - state: current ground-truth robot state.
     *
     * Output:
     *   SensorReadings containing encoder and IMU/gyro values.
     */
    SensorReadings sample(double time_s, const RobotState& state) override;

private:
    static constexpr std::size_t kMaxBufferedSamples = 512U;

    /// One stored truth sample used to implement delayed measurements.
    struct SampleRecord {
        double timestamp_s{0.0};
        double value{0.0};
    };

    /**
     * Per-channel state needed to apply drift, delay, and sample hold behavior.
     */
    struct ChannelState {
        std::deque<SampleRecord> history{};
        double accumulated_drift{0.0};
        double last_output{0.0};
        bool has_last_output{false};
    };

    SensorParams sensor_params_{};
    ScenarioParams scenario_{};
    RandomGenerator rng_{};
    double previous_sample_time_s_{0.0};
    bool has_previous_sample_{false};
    ChannelState left_encoder_state_{};
    ChannelState right_encoder_state_{};
    ChannelState imu_heading_state_{};
    ChannelState gyro_state_{};

    /// Returns a truth sample delayed by the configured channel latency.
    double delayed_truth(
        double time_s,
        double true_value,
        const SensorChannelErrorModel& model,
        ChannelState& channel_state) const;

    /// Applies bias, drift, noise, corruption, and quantization to a truth value.
    double apply_error_model(
        double time_step_s,
        double delayed_truth_value,
        const SensorChannelErrorModel& model,
        ChannelState& channel_state,
        double extra_bias,
        double extra_drift_per_second);

    /// Pushes one true sample into the bounded delay-history buffer.
    void push_history(ChannelState& channel_state, double time_s, double true_value);
};

/**
 * Simple differential-drive odometry estimator based on encoders and IMU.
 *
 * Layer:
 *   Estimation.
 *
 * Assumptions:
 *   - Pure rolling kinematics
 *   - Planar motion
 *   - IMU heading, when valid, is used as the preferred heading source
 *
 * Not Modeled:
 *   - Slip compensation
 *   - Full covariance propagation
 *   - Sensor fusion with external localization sources
 */
class WheelOdometryEstimator final : public StateEstimator {
public:
    explicit WheelOdometryEstimator(double track_width_m);

    void reset(const ScenarioParams& scenario, const RobotState& initial_state) override;

    /**
     * Updates the pose estimate from wheel travel and heading measurements.
     *
     * Output:
     *   EstimatedState in SI units [m, rad, m/s, rad/s].
     */
    EstimatedState estimate(
        double time_s,
        const RobotState& true_state,
        const SensorReadings& sensors) override;

private:
    double track_width_m_{0.30};
    double previous_timestamp_s_{0.0};
    double previous_left_encoder_m_{0.0};
    double previous_right_encoder_m_{0.0};
    double previous_heading_rad_{0.0};
    bool initialized_{false};
    EstimatedState estimate_{};
};

}  // namespace robot_sim
