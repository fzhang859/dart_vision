#ifndef DART_VISION_DETECTOR_DETECTOR_NODE_HPP
#define DART_VISION_DETECTOR_DETECTOR_NODE_HPP

#include <memory>
#include <mutex>
#include <optional>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "dart_vision_detector/angle_filter.hpp"
#include "dart_vision_detector/light_detector.hpp"
#include "dart_vision_detector/pnp_solver.hpp"
#include "dart_vision_interfaces/msg/light_detection.hpp"

namespace dart_vision::detector {

/**
 * @brief 串联目标检测、PnP 位姿估计和角度滤波的 ROS 2 节点。
 *
 * 节点从 image_topic 接收图像，从 camera_info_topic 获取标定参数，并向
 * detection_topic 发布检测结果。光源检测参数支持运行时原子更新；话题、PnP 和
 * 滤波参数只在启动时加载。
 */
class DetectorNode : public rclcpp::Node {
public:
    explicit DetectorNode(const rclcpp::NodeOptions& options);

private:
    void declareParameters();

    [[nodiscard]] LightDetectorConfig readDetectorConfig() const;
    [[nodiscard]] PnpSolverConfig readPnpSolverConfig() const;
    [[nodiscard]] AngleFilterConfig readAngleFilterConfig() const;

    [[nodiscard]] rcl_interfaces::msg::SetParametersResult
    onParametersChanged(const std::vector<rclcpp::Parameter>& parameters);

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg);

    void publishDetection(const std_msgs::msg::Header& header,
                          const LightDetectionResult& result,
                          const std::optional<PnpResult>& pnp_result,
                          const std::optional<double>& filtered_yaw_rad,
                          const std::optional<double>& filtered_pitch_rad);

    std::string image_topic_;
    std::string camera_info_topic_;
    std::string detection_topic_;
    std::string debug_mask_topic_;
    bool publish_debug_mask_{};

    mutable std::mutex detector_mutex_;
    LightDetectorConfig detector_config_;
    std::shared_ptr<LightDetector> detector_;
    PnpSolverConfig pnp_solver_config_;
    AngleFilterConfig angle_filter_config_;
    std::shared_ptr<PnpSolver> pnp_solver_;
    std::unique_ptr<AngleFilter> yaw_filter_;
    std::unique_ptr<AngleFilter> pitch_filter_;
    std::optional<rclcpp::Time> last_detection_stamp_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Publisher<dart_vision_interfaces::msg::LightDetection>::SharedPtr detection_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_mask_publisher_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
} // namespace dart_vision::detector

#endif // DART_VISION_DETECTOR_DETECTOR_NODE_HPP
