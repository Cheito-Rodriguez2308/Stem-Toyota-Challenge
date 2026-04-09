/*
 * Purpose:
 *   Implements the sensor simulation and wheel-odometry estimator.
 *
 * Role In The Architecture:
 *   This translation unit forms the bridge between the true plant state and
 *   the controller-facing estimated state. It is intentionally separate from
 *   plant dynamics so truth, measurements, and estimates remain distinct.
 *
 * Key Functions:
 *   - DifferentialDriveSensorSimulator::sample
 *   - DifferentialDriveSensorSimulator::apply_error_model
 *   - WheelOdometryEstimator::estimate
 *
 * Dependencies:
 *   - sensor_simulator.hpp
 *   - math_utils.hpp for angle wrapping and kinematic integration
 */

#include "sensor_simulator.hpp"

#include "math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot_sim {

DifferentialDriveSensorSimulator::DifferentialDriveSensorSimulator(
    SensorParams sensor_params,
    std::uint32_t seed)
    : sensor_params_(std::move(sensor_params)), rng_(seed) {}

void DifferentialDriveSensorSimulator::reset(
    const ScenarioParams& scenario,
    const RobotState& initial_state) {
    SensorModel::reset(scenario, initial_state);
    scenario_ = scenario;
    previous_sample_time_s_ = initial_state.sim_time_s;
    has_previous_sample_ = false;
    left_encoder_state_ = {};
    right_encoder_state_ = {};
    imu_heading_state_ = {};
    gyro_state_ = {};

    push_history(left_encoder_state_, initial_state.sim_time_s, initial_state.left_wheel_position_m);
    push_history(right_encoder_state_, initial_state.sim_time_s, initial_state.right_wheel_position_m);
    push_history(imu_heading_state_, initial_state.sim_time_s, initial_state.pose.theta_rad);
    push_history(gyro_state_, initial_state.sim_time_s, initial_state.angular_velocity_radps);

    left_encoder_state_.last_output = initial_state.left_wheel_position_m;
    right_encoder_state_.last_output = initial_state.right_wheel_position_m;
    imu_heading_state_.last_output = initial_state.pose.theta_rad;
    gyro_state_.last_output = initial_state.angular_velocity_radps;
    left_encoder_state_.has_last_output = true;
    right_encoder_state_.has_last_output = true;
    imu_heading_state_.has_last_output = true;
    gyro_state_.has_last_output = true;
}

SensorReadings DifferentialDriveSensorSimulator::sample(double time_s, const RobotState& state) {
    const double time_step_s = has_previous_sample_
        ? std::max(0.0, time_s - previous_sample_time_s_)
        : 0.0;
    has_previous_sample_ = true;
    previous_sample_time_s_ = time_s;

    push_history(left_encoder_state_, time_s, state.left_wheel_position_m);
    push_history(right_encoder_state_, time_s, state.right_wheel_position_m);
    push_history(imu_heading_state_, time_s, state.pose.theta_rad);
    push_history(gyro_state_, time_s, state.angular_velocity_radps);

    SensorReadings readings{};
    readings.timestamp_s = time_s;

    const double left_truth_m =
        delayed_truth(time_s, state.left_wheel_position_m, sensor_params_.left_encoder, left_encoder_state_);
    const double right_truth_m =
        delayed_truth(time_s, state.right_wheel_position_m, sensor_params_.right_encoder, right_encoder_state_);
    const double imu_truth_rad =
        delayed_truth(time_s, state.pose.theta_rad, sensor_params_.imu_heading, imu_heading_state_);
    const double gyro_truth_radps =
        delayed_truth(time_s, state.angular_velocity_radps, sensor_params_.gyro, gyro_state_);

    readings.left_encoder_m = apply_error_model(
        time_step_s,
        left_truth_m,
        sensor_params_.left_encoder,
        left_encoder_state_,
        0.0,
        0.0);
    readings.right_encoder_m = apply_error_model(
        time_step_s,
        right_truth_m,
        sensor_params_.right_encoder,
        right_encoder_state_,
        0.0,
        0.0);
    readings.imu_heading_rad = apply_error_model(
        time_step_s,
        imu_truth_rad,
        sensor_params_.imu_heading,
        imu_heading_state_,
        scenario_.enable_sensor_drift ? scenario_.imu_bias_override_rad : 0.0,
        scenario_.enable_sensor_drift ? scenario_.imu_drift_override_radps : 0.0);
    readings.gyro_z_radps = apply_error_model(
        time_step_s,
        gyro_truth_radps,
        sensor_params_.gyro,
        gyro_state_,
        0.0,
        scenario_.enable_sensor_drift ? scenario_.imu_drift_override_radps : 0.0);

    // Engineering Notes:
    //   Each channel starts from delayed ground truth and then applies
    //   measurement = truth_delayed + bias + integral(drift dt) + noise
    //   followed by optional corruption and quantization.
    // Assumptions:
    //   - Error channels are independent
    //   - Noise is white and Gaussian
    //   - Drift is a simple linear bias walk, not a calibrated stochastic process
    // What Could Break In Reality:
    //   - Correlated timing faults across sensors
    //   - Vibration- or temperature-driven IMU changes
    //   - Wheel slip causing encoder-based odometry error in ways not captured
    //     by this additive sensor model alone
    if (sensor_params_.enable_dropout &&
        sensor_params_.left_encoder.dropout_probability > 0.0 &&
        rng_.bernoulli(sensor_params_.left_encoder.dropout_probability)) {
        readings.left_encoder_valid = false;
        if (left_encoder_state_.has_last_output) {
            readings.left_encoder_m = left_encoder_state_.last_output;
        }
    } else {
        readings.left_encoder_valid = true;
        left_encoder_state_.last_output = readings.left_encoder_m;
        left_encoder_state_.has_last_output = true;
    }

    if (sensor_params_.enable_dropout &&
        sensor_params_.right_encoder.dropout_probability > 0.0 &&
        rng_.bernoulli(sensor_params_.right_encoder.dropout_probability)) {
        readings.right_encoder_valid = false;
        if (right_encoder_state_.has_last_output) {
            readings.right_encoder_m = right_encoder_state_.last_output;
        }
    } else {
        readings.right_encoder_valid = true;
        right_encoder_state_.last_output = readings.right_encoder_m;
        right_encoder_state_.has_last_output = true;
    }

    if (sensor_params_.enable_dropout &&
        sensor_params_.imu_heading.dropout_probability > 0.0 &&
        rng_.bernoulli(sensor_params_.imu_heading.dropout_probability)) {
        readings.imu_valid = false;
        if (imu_heading_state_.has_last_output) {
            readings.imu_heading_rad = imu_heading_state_.last_output;
        }
    } else {
        readings.imu_valid = true;
        readings.imu_heading_rad = math::wrap_angle(readings.imu_heading_rad);
        imu_heading_state_.last_output = readings.imu_heading_rad;
        imu_heading_state_.has_last_output = true;
    }

    if (sensor_params_.enable_dropout &&
        sensor_params_.gyro.dropout_probability > 0.0 &&
        rng_.bernoulli(sensor_params_.gyro.dropout_probability)) {
        readings.gyro_valid = false;
        if (gyro_state_.has_last_output) {
            readings.gyro_z_radps = gyro_state_.last_output;
        }
    } else {
        readings.gyro_valid = true;
        gyro_state_.last_output = readings.gyro_z_radps;
        gyro_state_.has_last_output = true;
    }

    return readings;
}

double DifferentialDriveSensorSimulator::delayed_truth(
    double time_s,
    double true_value,
    const SensorChannelErrorModel& model,
    ChannelState& channel_state) const {
    if (!sensor_params_.enable_delay || model.delay_s <= 0.0) {
        return true_value;
    }
    if (channel_state.history.empty()) {
        return true_value;
    }

    const double delayed_timestamp_s = time_s - model.delay_s;
    double candidate_value = channel_state.history.front().value;
    for (const auto& sample : channel_state.history) {
        if (sample.timestamp_s <= delayed_timestamp_s) {
            candidate_value = sample.value;
        } else {
            break;
        }
    }
    return candidate_value;
}

double DifferentialDriveSensorSimulator::apply_error_model(
    double time_step_s,
    double delayed_truth_value,
    const SensorChannelErrorModel& model,
    ChannelState& channel_state,
    double extra_bias,
    double extra_drift_per_second) {
    channel_state.accumulated_drift +=
        (model.drift_per_second + extra_drift_per_second) * time_step_s;

    double measurement =
        delayed_truth_value + model.static_bias + extra_bias + channel_state.accumulated_drift;
    if (model.gaussian_noise_stddev > 0.0) {
        measurement += rng_.normal(0.0, model.gaussian_noise_stddev);
    }

    if (sensor_params_.enable_corruption &&
        model.corruption_probability > 0.0 &&
        rng_.bernoulli(model.corruption_probability)) {
        measurement +=
            rng_.normal(0.0, model.corruption_scale * std::max(1.0, std::abs(delayed_truth_value)));
    }

    if (sensor_params_.enable_quantization && model.quantization_step > 0.0) {
        measurement = math::quantize(measurement, model.quantization_step);
    }

    return measurement;
}

void DifferentialDriveSensorSimulator::push_history(
    ChannelState& channel_state,
    double time_s,
    double true_value) {
    channel_state.history.push_back({time_s, true_value});

    while (channel_state.history.size() > kMaxBufferedSamples) {
        channel_state.history.pop_front();
    }
}

WheelOdometryEstimator::WheelOdometryEstimator(double track_width_m)
    : track_width_m_(track_width_m) {}

void WheelOdometryEstimator::reset(const ScenarioParams& scenario, const RobotState& initial_state) {
    StateEstimator::reset(scenario, initial_state);
    previous_timestamp_s_ = initial_state.sim_time_s;
    previous_left_encoder_m_ = initial_state.left_wheel_position_m;
    previous_right_encoder_m_ = initial_state.right_wheel_position_m;
    previous_heading_rad_ = initial_state.pose.theta_rad;
    estimate_.pose = initial_state.pose;
    estimate_.linear_velocity_mps = initial_state.linear_velocity_mps;
    estimate_.angular_velocity_radps = initial_state.angular_velocity_radps;
    estimate_.covariance_trace = 0.0;
    initialized_ = true;
}

EstimatedState WheelOdometryEstimator::estimate(
    double time_s,
    const RobotState& true_state,
    const SensorReadings& sensors) {
    if (!initialized_) {
        reset({}, true_state);
    }

    const double left_encoder_m =
        sensors.left_encoder_valid ? sensors.left_encoder_m : previous_left_encoder_m_;
    const double right_encoder_m =
        sensors.right_encoder_valid ? sensors.right_encoder_m : previous_right_encoder_m_;
    const double delta_left_m = left_encoder_m - previous_left_encoder_m_;
    const double delta_right_m = right_encoder_m - previous_right_encoder_m_;
    const double delta_center_m = 0.5 * (delta_left_m + delta_right_m);

    double heading_rad = estimate_.pose.theta_rad;
    double delta_heading_rad = 0.0;
    if (sensors.imu_valid) {
        heading_rad = sensors.imu_heading_rad;
        delta_heading_rad = math::wrap_angle(heading_rad - previous_heading_rad_);
    } else {
        delta_heading_rad = (delta_right_m - delta_left_m) / std::max(1e-6, track_width_m_);
        heading_rad = math::wrap_angle(estimate_.pose.theta_rad + delta_heading_rad);
    }

    const double midpoint_heading_rad =
        math::wrap_angle(estimate_.pose.theta_rad + 0.5 * delta_heading_rad);
    estimate_.pose.x_m += delta_center_m * std::cos(midpoint_heading_rad);
    estimate_.pose.y_m += delta_center_m * std::sin(midpoint_heading_rad);
    estimate_.pose.theta_rad = heading_rad;

    const double time_step_s = std::max(1e-6, time_s - previous_timestamp_s_);
    estimate_.linear_velocity_mps = delta_center_m / time_step_s;
    estimate_.angular_velocity_radps = delta_heading_rad / time_step_s;

    const double validity_penalty =
        (sensors.left_encoder_valid ? 0.0 : 1.0) +
        (sensors.right_encoder_valid ? 0.0 : 1.0) +
        (sensors.imu_valid ? 0.0 : 1.0);
    estimate_.covariance_trace = validity_penalty;

    previous_timestamp_s_ = time_s;
    previous_left_encoder_m_ = left_encoder_m;
    previous_right_encoder_m_ = right_encoder_m;
    previous_heading_rad_ = heading_rad;
    return estimate_;
}

}  // namespace robot_sim
