/*
 * Purpose:
 *   Implements deterministic random-number helpers used by sensor simulation
 *   and robustness studies.
 *
 * Role In The Architecture:
 *   This translation unit keeps stochastic behavior reproducible and localized
 *   instead of scattering distribution logic across many modules.
 *
 * Key Functions:
 *   - RandomGenerator::normal
 *   - RandomGenerator::uniform
 *   - RandomGenerator::bernoulli
 *
 * Dependencies:
 *   - random_utils.hpp
 *   - math_utils.hpp for pi in the Box-Muller transform
 */

#include "random_utils.hpp"

#include "math_utils.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace robot_sim {

RandomGenerator::RandomGenerator(std::uint32_t seed)
    : seed_(seed), engine_(seed) {}

void RandomGenerator::reseed(std::uint32_t seed) {
    seed_ = seed;
    engine_.seed(seed_);
    has_spare_normal_ = false;
    spare_normal_sample_ = 0.0;
}

std::uint32_t RandomGenerator::seed() const noexcept {
    return seed_;
}

double RandomGenerator::next_unit_interval() {
    // Combine two engine outputs into a high-resolution [0, 1) sample.
    const std::uint64_t high_bits = static_cast<std::uint64_t>(engine_() >> 5);
    const std::uint64_t low_bits = static_cast<std::uint64_t>(engine_() >> 6);
    return (static_cast<double>(high_bits) * 67108864.0 + static_cast<double>(low_bits)) /
           9007199254740992.0;
}

double RandomGenerator::uniform(double min_value, double max_value) {
    if (min_value > max_value) {
        throw std::invalid_argument("uniform requires min_value <= max_value.");
    }

    if (min_value == max_value) {
        return min_value;
    }

    return min_value + (max_value - min_value) * next_unit_interval();
}

int RandomGenerator::uniform_int(int min_value, int max_value) {
    if (min_value > max_value) {
        throw std::invalid_argument("uniform_int requires min_value <= max_value.");
    }

    const std::int64_t signed_range =
        static_cast<std::int64_t>(max_value) - static_cast<std::int64_t>(min_value) + 1LL;
    const std::uint64_t range = static_cast<std::uint64_t>(signed_range);
    const std::uint64_t engine_span = static_cast<std::uint64_t>(std::mt19937::max()) + 1ULL;
    const std::uint64_t acceptance_limit = engine_span - (engine_span % range);

    std::uint64_t raw_value = 0ULL;
    do {
        raw_value = static_cast<std::uint64_t>(engine_());
    } while (raw_value >= acceptance_limit);

    return min_value + static_cast<int>(raw_value % range);
}

double RandomGenerator::normal(double mean, double stddev) {
    if (stddev < 0.0) {
        throw std::invalid_argument("normal requires stddev >= 0.");
    }

    if (stddev == 0.0) {
        return mean;
    }

    if (has_spare_normal_) {
        has_spare_normal_ = false;
        return mean + stddev * spare_normal_sample_;
    }

    // Box-Muller transform:
    //   z0 = sqrt(-2 ln u1) cos(2 pi u2)
    //   z1 = sqrt(-2 ln u1) sin(2 pi u2)
    double uniform_1 = next_unit_interval();
    while (uniform_1 <= std::numeric_limits<double>::min()) {
        uniform_1 = next_unit_interval();
    }

    const double uniform_2 = next_unit_interval();
    const double radius = std::sqrt(-2.0 * std::log(uniform_1));
    const double angle_rad = 2.0 * math::kPi * uniform_2;

    spare_normal_sample_ = radius * std::sin(angle_rad);
    has_spare_normal_ = true;
    return mean + stddev * (radius * std::cos(angle_rad));
}

bool RandomGenerator::bernoulli(double probability_true) {
    if (probability_true < 0.0 || probability_true > 1.0) {
        throw std::invalid_argument("bernoulli requires probability in [0, 1].");
    }

    return next_unit_interval() < probability_true;
}

Eigen::Vector2d RandomGenerator::uniform_vector2d(
    const Eigen::Vector2d& min_values,
    const Eigen::Vector2d& max_values) {
    Eigen::Vector2d result{};
    result.x() = uniform(min_values.x(), max_values.x());
    result.y() = uniform(min_values.y(), max_values.y());
    return result;
}

std::mt19937& RandomGenerator::engine() noexcept {
    return engine_;
}

const std::mt19937& RandomGenerator::engine() const noexcept {
    return engine_;
}

}  // namespace robot_sim
