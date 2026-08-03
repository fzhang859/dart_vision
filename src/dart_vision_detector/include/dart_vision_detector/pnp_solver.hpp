#ifndef DART_VISION_DETECTOR_PNP_SOLVER_HPP
#define DART_VISION_DETECTOR_PNP_SOLVER_HPP

#include <array>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

#include "dart_vision_detector/light_detector.hpp"

namespace dart_vision::detector {

/// 基于相机标定参数和已知目标尺寸的平面 PnP 配置。
struct PnpSolverConfig {
    std::array<double, 9> camera_matrix{};       ///< 按行存储的 3x3 相机内参矩阵。
    std::vector<double> distortion_coefficients; ///< OpenCV 格式的镜头畸变系数。
    double target_radius_m{0.03};                ///< 圆形目标的实际半径。
    double max_reprojection_error_px{5.0}; ///< 接受 PnP 解的最大平均重投影误差。

    [[nodiscard]] bool isConfigValid() const;
};

/// 目标在相机光学坐标系中的位姿和瞄准角。
struct PnpResult {
    cv::Vec3d rotation_vector; ///< OpenCV Rodrigues 旋转向量。
    cv::Vec3d translation_m; ///< 目标相对相机的平移，+x 向右、+y 向下、+z 向前。
    double distance_m{}; ///< 相机原点到目标中心的欧氏距离。
    double yaw_rad{};    ///< 目标中心的水平偏航角，向右为正。
    double pitch_rad{};  ///< 目标中心的垂直俯仰角，向上为正。
    double reprojection_error_px{}; ///< 五个模型点的平均重投影误差。
};

class PnpSolver {
public:
    explicit PnpSolver(const PnpSolverConfig& config);

    /**
     * @brief 求解候选目标位姿。
     *
     * IPPE 产生多个解时，仅保留相机前方且重投影误差合格的解，并返回其中误差
     * 最小者。候选几何无效、OpenCV 求解失败或没有合格解时返回空值。
     *
     * @param candidate 待求解的光源候选。
     * @return 合格解中重投影误差最小的结果；求解失败时返回 std::nullopt。
     */
    [[nodiscard]] std::optional<PnpResult> solve(const LightCandidate& candidate) const;

    /**
     * @brief 仅根据去畸变后的目标中心计算偏航角和俯仰角。
     *
     * @param center_px 目标中心坐标，单位为像素。
     * @return (yaw, pitch)，单位为弧度；向右、向上分别为正。
     * @throws std::invalid_argument 中心坐标包含非有限值。
     * @throws std::runtime_error 去畸变未能产生唯一结果。
     */
    [[nodiscard]] cv::Point2d calculatePixelAngles(const cv::Point2f& center_px) const;

private:
    PnpSolverConfig config_;

    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;

    std::vector<cv::Point3f> object_points_;

    [[nodiscard]] std::vector<cv::Point2f> makeImagePoints(const LightCandidate& candidate) const;

    [[nodiscard]] double calculateReprojectionError(const std::vector<cv::Point2f>& image_points,
                                                    const cv::Mat& rotation_vector,
                                                    const cv::Mat& translation_vector) const;
};

} // namespace dart_vision::detector

#endif // DART_VISION_DETECTOR_CORE_PNP_SOLVER_HPP
