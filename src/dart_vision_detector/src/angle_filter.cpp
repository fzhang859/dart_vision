#include "dart_vision_detector/angle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace dart_vision::detector {

bool AngleFilterConfig::isConfigValid() const {
    if (!std::isfinite(process_noise_rad2_per_s) || process_noise_rad2_per_s < 0.0) {
        return false;
    }

    if (!std::isfinite(measurement_noise_rad2) || measurement_noise_rad2 <= 0.0) {
        return false;
    }

    if (!std::isfinite(initial_uncertainty_rad2) || initial_uncertainty_rad2 <= 0.0) {
        return false;
    }

    if (!std::isfinite(max_time_step_s) || max_time_step_s <= 0.0) {
        return false;
    }

    return true;
}

AngleFilter::AngleFilter(const AngleFilterConfig& config) : config_(config) {
    if (!config_.isConfigValid()) {
        throw std::invalid_argument("Invalid angle filter configuration.");
    }
}

void AngleFilter::reset(const double angle_rad) {
    if (!std::isfinite(angle_rad)) {
        throw std::invalid_argument("Initial angle must be finite.");
    }

    estimated_angle_rad_ = normalizeAngle(angle_rad);

    uncertainty_rad2_ = config_.initial_uncertainty_rad2;

    initialized_ = true;
}

void AngleFilter::reset() {
    estimated_angle_rad_ = 0.0;
    uncertainty_rad2_ = 0.0;
    initialized_ = false;
}

double AngleFilter::update(const double measured_angle_rad, const double time_step_s) {
    if (!std::isfinite(measured_angle_rad)) {
        throw std::invalid_argument("Measured angle must be finite.");
    }

    if (!std::isfinite(time_step_s) || time_step_s <= 0.0) {
        throw std::invalid_argument("Time step must be finite and positive.");
    }

    const double normalized_measurement = normalizeAngle(measured_angle_rad);

    if (!initialized_) {
        reset(normalized_measurement);
        return estimated_angle_rad_;
    }

    const double effective_time_step_s = std::min(time_step_s, config_.max_time_step_s);

    const double predicted_angle_rad = estimated_angle_rad_;

    const double predicted_uncertainty_rad2 =
        uncertainty_rad2_ + config_.process_noise_rad2_per_s * effective_time_step_s;

    const double kalman_gain =
        predicted_uncertainty_rad2 / (predicted_uncertainty_rad2 + config_.measurement_noise_rad2);

    const double innovation_rad = normalizeAngle(normalized_measurement - predicted_angle_rad);

    estimated_angle_rad_ = normalizeAngle(predicted_angle_rad + kalman_gain * innovation_rad);

    uncertainty_rad2_ = (1.0 - kalman_gain) * predicted_uncertainty_rad2;

    return estimated_angle_rad_;
}

bool AngleFilter::isInitialized() const {
    return initialized_;
}

double AngleFilter::estimatedAngleRad() const {
    if (!initialized_) {
        throw std::logic_error("Angle filter is not initialized.");
    }

    return estimated_angle_rad_;
}

double AngleFilter::uncertaintyRad2() const {
    if (!initialized_) {
        throw std::logic_error("Angle filter is not initialized.");
    }

    return uncertainty_rad2_;
}

double AngleFilter::normalizeAngle(const double angle_rad) {
    constexpr double kPi = 3.14159265358979323846;

    constexpr double kTwoPi = 2.0 * kPi;

    double normalized = std::remainder(angle_rad, kTwoPi);

    if (normalized >= kPi) {
        normalized -= kTwoPi;
    }

    return normalized;
}

} // namespace dart_vision::detector
