#include "dart_vision_detector/pnp_solver.hpp"

#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <stdexcept>

namespace dart_vision::detector {
namespace {
cv::Vec3d matToVec3d(const cv::Mat& matrix) {
    if (matrix.total() != 3) {
        throw std::invalid_argument("Rotation or translation vector must contain 3 elements.");
    }

    cv::Mat matrix_64f;
    matrix.convertTo(matrix_64f, CV_64F);

    const cv::Mat flattened = matrix_64f.reshape(1, 3);

    return cv::Vec3d{
        flattened.at<double>(0, 0), flattened.at<double>(1, 0), flattened.at<double>(2, 0)};
};

} // namespace

bool PnpSolverConfig::isConfigValid() const {
    for (const double value : camera_matrix) {
        if (!std::isfinite(value)) {
            return false;
        }
    }

    const double fx = camera_matrix[0];
    const double fy = camera_matrix[4];

    if (fx <= 0.0 || fy <= 0.0) {
        return false;
    }

    for (const double coefficient : distortion_coefficients) {
        if (!std::isfinite(coefficient)) {
            return false;
        }
    }

    if (!std::isfinite(target_radius_m) || target_radius_m <= 0.0) {
        return false;
    }

    if (!std::isfinite(max_reprojection_error_px) || max_reprojection_error_px <= 0.0) {
        return false;
    }

    return true;
}

PnpSolver::PnpSolver(const PnpSolverConfig& config) : config_(config) {
    if (!config_.isConfigValid()) {
        throw std::invalid_argument("Invalid PnP solver configuration.");
    }

    camera_matrix_ = cv::Mat(3, 3, CV_64F, config_.camera_matrix.data()).clone();

    if (config_.distortion_coefficients.empty()) {
        distortion_coefficients_ = cv::Mat();
    } else {
        distortion_coefficients_ = cv::Mat(config_.distortion_coefficients, true).reshape(1, 1);
        distortion_coefficients_.convertTo(distortion_coefficients_, CV_64F);
    }

    const float radius_m = static_cast<float>(config_.target_radius_m);
    object_points_ = {cv::Point3f(0.0f, 0.0f, 0.0f),
                      cv::Point3f(0.0f, -radius_m, 0.0f),
                      cv::Point3f(radius_m, 0.0f, 0.0f),
                      cv::Point3f(0.0f, radius_m, 0.0f),
                      cv::Point3f(-radius_m, 0.0f, 0.0f)};
}

std::vector<cv::Point2f> PnpSolver::makeImagePoints(const LightCandidate& candidate) const {
    if (!std::isfinite(candidate.center_px.x) || !std::isfinite(candidate.center_px.y) ||
        !std::isfinite(candidate.radius_px) || candidate.radius_px <= 0.0) {
        throw std::invalid_argument("Light candidate contains invalid geometry.");
    }

    const float center_x = candidate.center_px.x;
    const float center_y = candidate.center_px.y;
    const float radius_px = static_cast<float>(candidate.radius_px);

    return {cv::Point2f(center_x, center_y),
            cv::Point2f(center_x, center_y - radius_px),
            cv::Point2f(center_x + radius_px, center_y),
            cv::Point2f(center_x, center_y + radius_px),
            cv::Point2f(center_x - radius_px, center_y)};
}

double PnpSolver::calculateReprojectionError(const std::vector<cv::Point2f>& image_points,
                                             const cv::Mat& rotation_vector,
                                             const cv::Mat& translation_vector) const {
    std::vector<cv::Point2f> projected_points;

    cv::projectPoints(object_points_,
                      rotation_vector,
                      translation_vector,
                      camera_matrix_,
                      distortion_coefficients_,
                      projected_points);

    if (projected_points.size() != image_points.size() || projected_points.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double total_error_px = 0.0;

    for (std::size_t index = 0; index < image_points.size(); ++index) {
        total_error_px += cv::norm(projected_points[index] - image_points[index]);
    }

    return total_error_px / static_cast<double>(image_points.size());
}

std::optional<PnpResult> PnpSolver::solve(const LightCandidate& candidate) const {
    std::vector<cv::Point2f> image_points;

    try {
        image_points = makeImagePoints(candidate);
    } catch (const std::invalid_argument&) {
        return std::nullopt;
    }

    std::vector<cv::Mat> rotation_vectors;
    std::vector<cv::Mat> translation_vectors;

    bool solved = false;

    try {
        solved = cv::solvePnPGeneric(object_points_,
                                     image_points,
                                     camera_matrix_,
                                     distortion_coefficients_,
                                     rotation_vectors,
                                     translation_vectors,
                                     false,
                                     cv::SOLVEPNP_IPPE);
    } catch (const cv::Exception&) {
        return std::nullopt;
    }

    if (!solved || rotation_vectors.empty() ||
        rotation_vectors.size() != translation_vectors.size()) {
        return std::nullopt;
    }

    std::optional<PnpResult> best_result;

    for (std::size_t index = 0; index < rotation_vectors.size(); ++index) {
        const cv::Vec3d rotation_vector = matToVec3d(rotation_vectors[index]);

        const cv::Vec3d translation_m = matToVec3d(translation_vectors[index]);

        if (!std::isfinite(translation_m[0]) || !std::isfinite(translation_m[1]) ||
            !std::isfinite(translation_m[2]) || translation_m[2] <= 0.0) {
            continue;
        }

        const double reprojection_error_px = calculateReprojectionError(
            image_points, rotation_vectors[index], translation_vectors[index]);

        if (!std::isfinite(reprojection_error_px) ||
            reprojection_error_px > config_.max_reprojection_error_px) {
            continue;
        }

        const double x_m = translation_m[0];
        const double y_m = translation_m[1];
        const double z_m = translation_m[2];

        const double distance_m = cv::norm(translation_m);

        const double yaw_rad = std::atan2(x_m, z_m);

        const double pitch_rad = std::atan2(-y_m, std::hypot(x_m, z_m));

        const PnpResult result{
            rotation_vector, translation_m, distance_m, yaw_rad, pitch_rad, reprojection_error_px};

        if (!best_result || result.reprojection_error_px < best_result->reprojection_error_px) {
            best_result = result;
        }
    }

    return best_result;
}

cv::Point2d PnpSolver::calculatePixelAngles(const cv::Point2f& center_px) const {
    if (!std::isfinite(center_px.x) || !std::isfinite(center_px.y)) {
        throw std::invalid_argument("Target center must contain finite coordinates.");
    }

    std::vector<cv::Point2f> distorted_points{center_px};

    std::vector<cv::Point2f> normalized_points;

    cv::undistortPoints(
        distorted_points, normalized_points, camera_matrix_, distortion_coefficients_);

    if (normalized_points.size() != 1) {
        throw std::runtime_error("Failed to undistort target center.");
    }

    const double normalized_x = normalized_points[0].x;

    const double normalized_y = normalized_points[0].y;

    const double yaw_rad = std::atan2(normalized_x, 1.0);

    const double pitch_rad = std::atan2(-normalized_y, std::hypot(normalized_x, 1.0));

    return cv::Point2d{yaw_rad, pitch_rad};
}

} // namespace dart_vision::detector
