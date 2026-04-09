#pragma once

/*
 * Purpose:
 *   Declares small deterministic math helpers used throughout the repository.
 *
 * Role In The Architecture:
 *   These utilities keep simulation, control, estimation, and reporting code
 *   focused on engineering intent rather than repeated scalar boilerplate.
 *
 * Key Functions:
 *   - clamp
 *   - wrap_angle
 *   - quantize
 *   - saturate_rate
 *   - euclidean_distance
 *   - mean / rms / stddev
 *
 * Dependencies:
 *   - core_types.hpp for Pose2D and Eigen-based helpers
 */

#include "core_types.hpp"

#include <vector>

namespace robot_sim::math {

/// Mathematical constant pi [rad].
constexpr double kPi = 3.14159265358979323846;

/// Clamps a scalar to an inclusive interval [min_value, max_value].
double clamp(double value, double min_value, double max_value);

/// Wraps an angle into the interval [-pi, pi] [rad].
double wrap_angle(double angle_rad);

/// Applies uniform scalar quantization with the provided step size.
double quantize(double value, double step);

/// Limits the change between two consecutive values to max_delta.
double saturate_rate(double requested_value, double previous_value, double max_delta);

/// Computes planar distance between two poses using x and y only [m].
double euclidean_distance(const Pose2D& a, const Pose2D& b);

/// Computes planar distance between a pose and a 2D target [m].
double euclidean_distance(const Pose2D& pose, const Eigen::Vector2d& target);

/// Computes the arithmetic mean of a scalar sample set.
double mean(const std::vector<double>& values);

/// Computes the root-mean-square magnitude of a scalar sample set.
double rms(const std::vector<double>& values);

/// Computes population standard deviation of a scalar sample set.
double stddev(const std::vector<double>& values);

}  // namespace robot_sim::math
