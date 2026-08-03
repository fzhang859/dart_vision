#include "dart_vision_detector/detector_node.hpp"

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <functional>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <utility>

namespace dart_vision::detector {
namespace {
constexpr char kParameterPrefix[] = "light_detector.";

rcl_interfaces::msg::SetParametersResult parameterFailure(const std::string& reason) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = reason;
    return result;
}
} // namespace

DetectorNode::DetectorNode(const rclcpp::NodeOptions& options) : Node("detector_node", options) {
    declareParameters();
    detector_config_ = readDetectorConfig();
    if (!detector_config_.isConfigValid()) {
        throw std::invalid_argument("Initial light detector parameters are invalid");
    }
    detector_ = std::make_shared<LightDetector>(detector_config_);

    pnp_solver_config_ = readPnpSolverConfig();
    angle_filter_config_ = readAngleFilterConfig();
    if (!angle_filter_config_.isConfigValid()) {
        throw std::invalid_argument("Initial angle filter parameters are invalid");
    }
    yaw_filter_ = std::make_unique<AngleFilter>(angle_filter_config_);
    pitch_filter_ = std::make_unique<AngleFilter>(angle_filter_config_);

    image_topic_ = get_parameter("image_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    detection_topic_ = get_parameter("detection_topic").as_string();
    debug_mask_topic_ = get_parameter("debug_mask_topic").as_string();
    publish_debug_mask_ = get_parameter("publish_debug_mask").as_bool();

    detection_publisher_ = create_publisher<dart_vision_interfaces::msg::LightDetection>(
        detection_topic_, rclcpp::SensorDataQoS());
    debug_mask_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(debug_mask_topic_, rclcpp::SensorDataQoS());
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&DetectorNode::imageCallback, this, std::placeholders::_1));
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&DetectorNode::cameraInfoCallback, this, std::placeholders::_1));

    parameter_callback_ = add_on_set_parameters_callback(
        std::bind(&DetectorNode::onParametersChanged, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Detector listening on '%s', publishing detections on '%s'",
                image_topic_.c_str(),
                detection_topic_.c_str());
}

void DetectorNode::declareParameters() {
    const LightDetectorConfig defaults;
    const PnpSolverConfig pnp_defaults;
    const AngleFilterConfig filter_defaults;
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    read_only.description = "Loaded at startup; restart the node to change this parameter";

    declare_parameter<std::string>("image_topic", "image_raw");
    declare_parameter<std::string>("camera_info_topic", "camera_info", read_only);
    declare_parameter<std::string>("detection_topic", "light_detection");
    declare_parameter<std::string>("debug_mask_topic", "debug/binary_mask");
    declare_parameter<bool>("publish_debug_mask", false);

    declare_parameter<double>("light_detector.min_hue", defaults.min_hue);
    declare_parameter<double>("light_detector.max_hue", defaults.max_hue);
    declare_parameter<double>("light_detector.min_saturation", defaults.min_saturation);
    declare_parameter<double>("light_detector.min_value", defaults.min_value);
    declare_parameter<double>("light_detector.min_green_excess", defaults.min_green_excess);
    declare_parameter<double>("light_detector.min_area_px2", defaults.min_area_px2);
    declare_parameter<double>("light_detector.min_radius_px", defaults.min_radius_px);
    declare_parameter<double>("light_detector.max_radius_px", defaults.max_radius_px);
    declare_parameter<double>("light_detector.min_circularity", defaults.min_circularity);
    declare_parameter<double>("light_detector.min_aspect_ratio", defaults.min_aspect_ratio);
    declare_parameter<double>("light_detector.max_aspect_ratio", defaults.max_aspect_ratio);
    declare_parameter<double>("light_detector.min_fill_ratio", defaults.min_fill_ratio);
    declare_parameter<double>("light_detector.max_fill_ratio", defaults.max_fill_ratio);
    declare_parameter<double>("light_detector.min_inner_brightness", defaults.min_inner_brightness);
    declare_parameter<double>("light_detector.min_contrast_ratio", defaults.min_contrast_ratio);
    declare_parameter<int>("light_detector.cleanup.close_kernel_size",
                           defaults.cleanup.close_kernel_size);
    declare_parameter<int>("light_detector.cleanup.close_iterations",
                           defaults.cleanup.close_iterations);
    declare_parameter<int>("light_detector.cleanup.open_kernel_size",
                           defaults.cleanup.open_kernel_size);
    declare_parameter<int>("light_detector.cleanup.open_iterations",
                           defaults.cleanup.open_iterations);
    declare_parameter<bool>("light_detector.cleanup.fill_holes", defaults.cleanup.fill_holes);
    declare_parameter<bool>("light_detector.cleanup.remove_small_speckle",
                            defaults.cleanup.remove_small_speckle);
    declare_parameter<double>("light_detector.cleanup.max_speckle_area_px2",
                              defaults.cleanup.max_speckle_area_px2);

    declare_parameter<double>(
        "pnp_solver.target_radius_m", pnp_defaults.target_radius_m, read_only);
    declare_parameter<double>(
        "pnp_solver.max_reprojection_error_px", pnp_defaults.max_reprojection_error_px, read_only);
    declare_parameter<double>("angle_filter.process_noise_rad2_per_s",
                              filter_defaults.process_noise_rad2_per_s,
                              read_only);
    declare_parameter<double>(
        "angle_filter.measurement_noise_rad2", filter_defaults.measurement_noise_rad2, read_only);
    declare_parameter<double>("angle_filter.initial_uncertainty_rad2",
                              filter_defaults.initial_uncertainty_rad2,
                              read_only);
    declare_parameter<double>(
        "angle_filter.max_time_step_s", filter_defaults.max_time_step_s, read_only);
}

LightDetectorConfig DetectorNode::readDetectorConfig() const {
    LightDetectorConfig config;
    config.min_hue = get_parameter("light_detector.min_hue").as_double();
    config.max_hue = get_parameter("light_detector.max_hue").as_double();
    config.min_saturation = get_parameter("light_detector.min_saturation").as_double();
    config.min_value = get_parameter("light_detector.min_value").as_double();
    config.min_green_excess = get_parameter("light_detector.min_green_excess").as_double();
    config.min_area_px2 = get_parameter("light_detector.min_area_px2").as_double();
    config.min_radius_px = get_parameter("light_detector.min_radius_px").as_double();
    config.max_radius_px = get_parameter("light_detector.max_radius_px").as_double();
    config.min_circularity = get_parameter("light_detector.min_circularity").as_double();
    config.min_aspect_ratio = get_parameter("light_detector.min_aspect_ratio").as_double();
    config.max_aspect_ratio = get_parameter("light_detector.max_aspect_ratio").as_double();
    config.min_fill_ratio = get_parameter("light_detector.min_fill_ratio").as_double();
    config.max_fill_ratio = get_parameter("light_detector.max_fill_ratio").as_double();
    config.min_inner_brightness = get_parameter("light_detector.min_inner_brightness").as_double();
    config.min_contrast_ratio = get_parameter("light_detector.min_contrast_ratio").as_double();
    config.cleanup.close_kernel_size =
        static_cast<int>(get_parameter("light_detector.cleanup.close_kernel_size").as_int());
    config.cleanup.close_iterations =
        static_cast<int>(get_parameter("light_detector.cleanup.close_iterations").as_int());
    config.cleanup.open_kernel_size =
        static_cast<int>(get_parameter("light_detector.cleanup.open_kernel_size").as_int());
    config.cleanup.open_iterations =
        static_cast<int>(get_parameter("light_detector.cleanup.open_iterations").as_int());
    config.cleanup.fill_holes = get_parameter("light_detector.cleanup.fill_holes").as_bool();
    config.cleanup.remove_small_speckle =
        get_parameter("light_detector.cleanup.remove_small_speckle").as_bool();
    config.cleanup.max_speckle_area_px2 =
        get_parameter("light_detector.cleanup.max_speckle_area_px2").as_double();
    return config;
}

PnpSolverConfig DetectorNode::readPnpSolverConfig() const {
    PnpSolverConfig config;
    config.target_radius_m = get_parameter("pnp_solver.target_radius_m").as_double();
    config.max_reprojection_error_px =
        get_parameter("pnp_solver.max_reprojection_error_px").as_double();
    return config;
}

AngleFilterConfig DetectorNode::readAngleFilterConfig() const {
    AngleFilterConfig config;
    config.process_noise_rad2_per_s =
        get_parameter("angle_filter.process_noise_rad2_per_s").as_double();
    config.measurement_noise_rad2 =
        get_parameter("angle_filter.measurement_noise_rad2").as_double();
    config.initial_uncertainty_rad2 =
        get_parameter("angle_filter.initial_uncertainty_rad2").as_double();
    config.max_time_step_s = get_parameter("angle_filter.max_time_step_s").as_double();
    return config;
}

rcl_interfaces::msg::SetParametersResult
DetectorNode::onParametersChanged(const std::vector<rclcpp::Parameter>& parameters) {
    LightDetectorConfig updated;
    bool updated_publish_debug_mask = false;
    {
        std::lock_guard<std::mutex> lock(detector_mutex_);
        updated = detector_config_;
        updated_publish_debug_mask = publish_debug_mask_;
    }

    try {
        for (const auto& parameter : parameters) {
            const std::string& name = parameter.get_name();
            if (name.rfind(kParameterPrefix, 0) != 0) {
                if (name == "publish_debug_mask") {
                    updated_publish_debug_mask = parameter.as_bool();
                } else if (name == "image_topic" || name == "camera_info_topic" ||
                           name == "detection_topic" || name == "debug_mask_topic") {
                    return parameterFailure(name + " cannot be changed while the node is running");
                }
                continue;
            }

            // clang-format off
            #define UPDATE_DOUBLE(field, parameter_name) \
                if (name == parameter_name) {            \
                    updated.field = parameter.as_double(); \
                    continue;                            \
                }
            UPDATE_DOUBLE(min_hue, "light_detector.min_hue")
            UPDATE_DOUBLE(max_hue, "light_detector.max_hue")
            UPDATE_DOUBLE(min_saturation, "light_detector.min_saturation")
            UPDATE_DOUBLE(min_value, "light_detector.min_value")
            UPDATE_DOUBLE(min_green_excess, "light_detector.min_green_excess")
            UPDATE_DOUBLE(min_area_px2, "light_detector.min_area_px2")
            UPDATE_DOUBLE(min_radius_px, "light_detector.min_radius_px")
            UPDATE_DOUBLE(max_radius_px, "light_detector.max_radius_px")
            UPDATE_DOUBLE(min_circularity, "light_detector.min_circularity")
            UPDATE_DOUBLE(min_aspect_ratio, "light_detector.min_aspect_ratio")
            UPDATE_DOUBLE(max_aspect_ratio, "light_detector.max_aspect_ratio")
            UPDATE_DOUBLE(min_fill_ratio, "light_detector.min_fill_ratio")
            UPDATE_DOUBLE(max_fill_ratio, "light_detector.max_fill_ratio")
            UPDATE_DOUBLE(min_inner_brightness, "light_detector.min_inner_brightness")
            UPDATE_DOUBLE(min_contrast_ratio, "light_detector.min_contrast_ratio")
            UPDATE_DOUBLE(cleanup.max_speckle_area_px2,
                          "light_detector.cleanup.max_speckle_area_px2")
            #undef UPDATE_DOUBLE
            // clang-format on

            if (name == "light_detector.cleanup.close_kernel_size") {
                updated.cleanup.close_kernel_size = static_cast<int>(parameter.as_int());
            } else if (name == "light_detector.cleanup.close_iterations") {
                updated.cleanup.close_iterations = static_cast<int>(parameter.as_int());
            } else if (name == "light_detector.cleanup.open_kernel_size") {
                updated.cleanup.open_kernel_size = static_cast<int>(parameter.as_int());
            } else if (name == "light_detector.cleanup.open_iterations") {
                updated.cleanup.open_iterations = static_cast<int>(parameter.as_int());
            } else if (name == "light_detector.cleanup.fill_holes") {
                updated.cleanup.fill_holes = parameter.as_bool();
            } else if (name == "light_detector.cleanup.remove_small_speckle") {
                updated.cleanup.remove_small_speckle = parameter.as_bool();
            }
        }
    } catch (const rclcpp::ParameterTypeException& error) {
        return parameterFailure(error.what());
    }

    if (!updated.isConfigValid()) {
        return parameterFailure("Light detector parameter combination is invalid");
    }

    auto updated_detector = std::make_shared<LightDetector>(updated);
    {
        std::lock_guard<std::mutex> lock(detector_mutex_);
        detector_config_ = updated;
        detector_ = std::move(updated_detector);
        publish_debug_mask_ = updated_publish_debug_mask;
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
    std::shared_ptr<LightDetector> detector;
    std::shared_ptr<PnpSolver> pnp_solver;
    bool publish_debug_mask = false;
    {
        std::lock_guard<std::mutex> lock(detector_mutex_);
        detector = detector_;
        pnp_solver = pnp_solver_;
        publish_debug_mask = publish_debug_mask_;
    }

    try {
        const cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image_msg, "bgr8");
        const LightDetectionResult result = detector->detect(cv_image->image);
        std::optional<PnpResult> pnp_result;
        std::optional<double> filtered_yaw_rad;
        std::optional<double> filtered_pitch_rad;
        if (result.target && pnp_solver) {
            pnp_result = pnp_solver->solve(*result.target);
        }

        {
            std::lock_guard<std::mutex> lock(detector_mutex_);
            if (pnp_result) {
                const rclcpp::Time current_stamp(image_msg->header.stamp);
                double time_step_s = angle_filter_config_.max_time_step_s;
                if (last_detection_stamp_) {
                    const double measured_time_step_s =
                        (current_stamp - *last_detection_stamp_).seconds();
                    if (measured_time_step_s > 0.0) {
                        time_step_s = measured_time_step_s;
                    } else {
                        yaw_filter_->reset();
                        pitch_filter_->reset();
                    }
                }
                filtered_yaw_rad = yaw_filter_->update(pnp_result->yaw_rad, time_step_s);
                filtered_pitch_rad = pitch_filter_->update(pnp_result->pitch_rad, time_step_s);
                last_detection_stamp_ = current_stamp;
            } else {
                // 不对丢失目标进行运动外推；下一次有效检测将从新观测重新初始化。
                yaw_filter_->reset();
                pitch_filter_->reset();
                last_detection_stamp_.reset();
            }
        }

        publishDetection(
            image_msg->header, result, pnp_result, filtered_yaw_rad, filtered_pitch_rad);

        if (publish_debug_mask) {
            debug_mask_publisher_->publish(
                *cv_bridge::CvImage(image_msg->header, "mono8", result.binary_mask).toImageMsg());
        }
    } catch (const cv_bridge::Exception& error) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Image conversion failed: %s", error.what());
    } catch (const std::exception& error) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "Detection failed: %s", error.what());
    }
}

void DetectorNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg) {
    PnpSolverConfig config;
    {
        std::lock_guard<std::mutex> lock(detector_mutex_);
        config = pnp_solver_config_;
    }
    std::copy(camera_info_msg->k.begin(), camera_info_msg->k.end(), config.camera_matrix.begin());
    config.distortion_coefficients = camera_info_msg->d;

    if (!config.isConfigValid()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Received invalid camera information");
        return;
    }

    auto solver = std::make_shared<PnpSolver>(config);
    {
        std::lock_guard<std::mutex> lock(detector_mutex_);
        pnp_solver_config_ = std::move(config);
        pnp_solver_ = std::move(solver);
    }
}

void DetectorNode::publishDetection(const std_msgs::msg::Header& header,
                                    const LightDetectionResult& result,
                                    const std::optional<PnpResult>& pnp_result,
                                    const std::optional<double>& filtered_yaw_rad,
                                    const std::optional<double>& filtered_pitch_rad) {
    dart_vision_interfaces::msg::LightDetection message;
    message.header = header;
    message.detected = result.target.has_value();
    message.candidate_count = static_cast<std::uint32_t>(result.candidates.size());

    if (result.target) {
        const LightCandidate& target = *result.target;
        message.center_x_px = target.center_px.x;
        message.center_y_px = target.center_px.y;
        message.radius_px = static_cast<float>(target.radius_px);
        message.area_px2 = static_cast<float>(target.area_px2);
        message.circularity = static_cast<float>(target.circularity);
        message.aspect_ratio = static_cast<float>(target.aspect_ratio);
        message.fill_ratio = static_cast<float>(target.fill_ratio);
        message.mean_inner_brightness = static_cast<float>(target.mean_inner_brightness);
        message.mean_outer_brightness = static_cast<float>(target.mean_outer_brightness);
        message.contrast_ratio = static_cast<float>(target.contrast_ratio);
        message.mean_radial_error_px = static_cast<float>(target.mean_radial_error_px);
        message.fit_score = static_cast<float>(target.fit_score);
    }
    message.pnp_valid = pnp_result.has_value();
    if (pnp_result) {
        message.position_x_m = pnp_result->translation_m[0];
        message.position_y_m = pnp_result->translation_m[1];
        message.position_z_m = pnp_result->translation_m[2];
        message.distance_m = pnp_result->distance_m;
        message.yaw_rad = pnp_result->yaw_rad;
        message.pitch_rad = pnp_result->pitch_rad;
        message.filtered_yaw_rad = filtered_yaw_rad.value_or(pnp_result->yaw_rad);
        message.filtered_pitch_rad = filtered_pitch_rad.value_or(pnp_result->pitch_rad);
        message.reprojection_error_px = pnp_result->reprojection_error_px;
    }
    detection_publisher_->publish(message);
}

} // namespace dart_vision::detector
