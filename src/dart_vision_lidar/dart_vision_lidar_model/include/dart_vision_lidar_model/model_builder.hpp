#ifndef DART_VISION_LIDAR_MODEL_MODEL_BUILDER_HPP
#define DART_VISION_LIDAR_MODEL_MODEL_BUILDER_HPP

#include <filesystem>
#include <string>
#include <vector>

#include "dart_vision_lidar_model/model_sampler.hpp"

namespace dart_vision::lidar {

enum class PcdEncoding {
    kBinary,
    kAscii,
};

struct ModelBuildEntry {
    std::string name;
    std::string role;
    std::filesystem::path input_ply;
    std::filesystem::path output_pcd;
    ModelSamplingOptions sampling;
    PcdEncoding pcd_encoding{PcdEncoding::kBinary};
    bool overwrite{false};
    bool enabled{true};
};

struct ModelBuildConfig {
    std::filesystem::path config_path;
    std::filesystem::path config_directory;
    std::vector<ModelBuildEntry> models;
};

struct ModelBuildRecord {
    std::string name;
    std::string role;
    std::filesystem::path output_pcd;
    bool skipped{false};
    ModelSamplingStats stats;
};

struct ModelBuildReport {
    std::vector<ModelBuildRecord> models;
    std::size_t built_count{0U};
    std::size_t skipped_count{0U};
};

/**
 * Loads and validates an ordinary (non-ROS-parameter) YAML file. Relative PLY
 * and PCD paths are resolved against the directory containing the YAML file.
 * Throws std::invalid_argument or std::runtime_error on invalid input.
 */
ModelBuildConfig loadModelBuildConfig(const std::filesystem::path& config_path);

/**
 * Builds all enabled entries. Disabled entries are recorded as skipped.
 * Output directories are created as needed. Throws on the first failed model.
 */
ModelBuildReport buildModels(const ModelBuildConfig& config);

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_MODEL_MODEL_BUILDER_HPP
