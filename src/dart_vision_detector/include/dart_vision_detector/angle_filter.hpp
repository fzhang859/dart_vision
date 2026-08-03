#ifndef DART_VISION_DETECTOR_ANGLE_FILTER_HPP
#define DART_VISION_DETECTOR_ANGLE_FILTER_HPP

namespace dart_vision::detector {

/// Kalman 滤波器参数配置
struct AngleFilterConfig {
    double process_noise_rad2_per_s{0.01}; ///< 单位时间内增加的过程噪声方差。
    double measurement_noise_rad2{0.0025}; ///< 单次角度观测的噪声方差。
    double initial_uncertainty_rad2{0.01}; ///< 初始化后的估计方差。
    double max_time_step_s{0.1};           ///< 预测阶段采用的最大时间间隔。

    [[nodiscard]] bool isConfigValid() const;
};

class AngleFilter {
public:
    explicit AngleFilter(const AngleFilterConfig& config);

    void reset(double angle_rad);

    void reset();

    /**
     * @brief 融合一次角度观测并返回新估计。
     *
     * 大于 max_time_step_s 的时间间隔会被截断，以限制长时间丢帧造成的不确定度增长。
     *
     * @param measured_angle_rad 测量角度，单位为弧度。
     * @param time_step_s 与上一次测量的时间间隔，单位为秒。
     * @return 更新后的角度估计，单位为弧度。
     * @throws std::invalid_argument 观测或时间间隔不合法。
     */
    [[nodiscard]] double update(double measured_angle_rad, double time_step_s);

    [[nodiscard]] bool isInitialized() const;

    [[nodiscard]] double estimatedAngleRad() const;

    [[nodiscard]] double uncertaintyRad2() const;

private:
    AngleFilterConfig config_;

    double estimated_angle_rad_{};
    double uncertainty_rad2_{};
    bool initialized_{false};

    [[nodiscard]] static double normalizeAngle(double angle_rad);
};

} // namespace dart_vision::detector

#endif // DART_VISION_DETECTOR_ANGLE_FILTER_HPP
