#pragma once

/*
 * Purpose:
 *   Declares the reproducible random-number utility used by sensor simulation
 *   and later Monte Carlo studies.
 *
 * Role In The Architecture:
 *   This file centralizes stochastic sampling so seeds remain explicit and
 *   reproducible behavior is easy to reason about during debugging and testing.
 *
 * Key Functions:
 *   - uniform / uniform_int
 *   - normal
 *   - bernoulli
 *   - uniform_vector2d
 *
 * Dependencies:
 *   - Eigen for 2D vector helpers
 *   - std::mt19937 as the underlying deterministic engine
 */

#include <Eigen/Dense>

#include <cstdint>
#include <random>

namespace robot_sim {

/**
 * Small wrapper around std::mt19937 with repository-specific helper methods.
 *
 * Layer:
 *   Utility / stochastic modeling support.
 *
 * Assumptions:
 *   Deterministic reproducibility matters more than distribution throughput or
 *   cryptographic quality.
 */
class RandomGenerator {
public:
    /// Constructs the generator with a reproducible seed.
    explicit RandomGenerator(std::uint32_t seed = 1U);

    /// Reinitializes the engine and clears cached distribution state.
    void reseed(std::uint32_t seed);

    /// Returns the current seed value.
    std::uint32_t seed() const noexcept;

    /// Samples a continuous uniform scalar on [min_value, max_value].
    double uniform(double min_value, double max_value);

    /// Samples a discrete uniform integer on [min_value, max_value].
    int uniform_int(int min_value, int max_value);

    /// Samples a Gaussian scalar with the provided mean and standard deviation.
    double normal(double mean, double stddev);

    /// Samples a Bernoulli event with probability_true in [0, 1].
    bool bernoulli(double probability_true);

    /// Samples each component of a 2D vector independently from uniform ranges.
    Eigen::Vector2d uniform_vector2d(
        const Eigen::Vector2d& min_values,
        const Eigen::Vector2d& max_values);

    /// Access to the underlying engine when a caller needs direct control.
    std::mt19937& engine() noexcept;

    /// Const access to the underlying engine.
    const std::mt19937& engine() const noexcept;

private:
    /**
     * Returns a high-resolution floating-point sample in [0, 1).
     *
     * Why:
     *   The repository uses this helper to keep normal-sampling behavior
     *   consistent across platforms.
     */
    double next_unit_interval();

    std::uint32_t seed_{1U};        ///< Seed used to initialize the engine.
    std::mt19937 engine_;           ///< Deterministic pseudo-random generator.
    bool has_spare_normal_{false};  ///< True when Box-Muller cached a second sample.
    double spare_normal_sample_{0.0}; ///< Cached standard-normal sample.
};

}  // namespace robot_sim
