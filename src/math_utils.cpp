/*
 * Purpose:
 *   Implements small reusable math helpers used across the repository.
 *
 * Role In The Architecture:
 *   These helpers reduce duplicated numerical code in the simulation,
 *   controller, estimator, and reporting layers.
 *
 * Key Functions:
 *   - wrap_angle
 *   - saturate_rate
 *   - rms / stddev
 *
 * Dependencies:
 *   - math_utils.hpp
 *   - standard algorithms and math functions
 */

#include "math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace robot_sim::math {

double clamp(double value, double min_value, double max_value) {
    if (min_value > max_value) {
        throw std::invalid_argument("clamp requires min_value <= max_value.");
    }

    return std::clamp(value, min_value, max_value);
}

double wrap_angle(double angle_rad) {
    // Wrapping to [-pi, pi] keeps heading errors on the shortest branch so the
    // controller does not try to rotate the long way around.
    while (angle_rad > kPi) {
        angle_rad -= 2.0 * kPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += 2.0 * kPi;
    }
    return angle_rad;
}

double quantize(double value, double step) {
    if (step <= 0.0) {
        return value;
    }

    return std::round(value / step) * step;
}

double saturate_rate(double requested_value, double previous_value, double max_delta) {
    if (max_delta < 0.0) {
        throw std::invalid_argument("saturate_rate requires max_delta >= 0.");
    }

    return clamp(requested_value, previous_value - max_delta, previous_value + max_delta);
}

double euclidean_distance(const Pose2D& a, const Pose2D& b) {
    const double delta_x_m = a.x_m - b.x_m;
    const double delta_y_m = a.y_m - b.y_m;
    return std::sqrt(delta_x_m * delta_x_m + delta_y_m * delta_y_m);
}

double euclidean_distance(const Pose2D& pose, const Eigen::Vector2d& target) {
    const double delta_x_m = pose.x_m - target.x();
    const double delta_y_m = pose.y_m - target.y();
    return std::sqrt(delta_x_m * delta_x_m + delta_y_m * delta_y_m);
}

double mean(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }

    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    return sum / static_cast<double>(values.size());
}

double rms(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }

    double sum_of_squares = 0.0;
    for (const double value : values) {
        sum_of_squares += value * value;
    }

    return std::sqrt(sum_of_squares / static_cast<double>(values.size()));
}

double stddev(const std::vector<double>& values) {
    if (values.size() < 2U) {
        return 0.0;
    }

    const double average = mean(values);
    double squared_error_sum = 0.0;
    for (const double value : values) {
        const double deviation = value - average;
        squared_error_sum += deviation * deviation;
    }

    return std::sqrt(squared_error_sum / static_cast<double>(values.size()));
}

}  // namespace robot_sim::math
