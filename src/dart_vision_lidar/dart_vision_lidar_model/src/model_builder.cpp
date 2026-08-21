#include "dart_vision_lidar_model/model_builder.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace dart_vision::lidar {
namespace {

constexpr int kSchemaVersion = 1;
constexpr double kQuaternionNormTolerance = 1.0e-3;

std::string describe(const YAML::Node& node, const std::string& context) {
    if (node.Mark().line >= 0) {
        return context + " (line " + std::to_string(node.Mark().line + 1) + ")";
    }
    return context;
}

YAML::Node
requireNode(const YAML::Node& parent, const std::string& key, const std::string& context) {
    const YAML::Node child = parent[key];
    if (!child.IsDefined()) {
        throw std::invalid_argument(context + ": missing required key '" + key + "'");
    }
    return child;
}

template <typename ValueT> ValueT parseScalar(const YAML::Node& node, const std::string& context) {
    if (!node.IsScalar()) {
        throw std::invalid_argument(describe(node, context) + " must be a scalar");
    }
    try {
        return node.as<ValueT>();
    } catch (const YAML::Exception& error) {
        throw std::invalid_argument(describe(node, context) + ": " + error.what());
    }
}

std::string parseNonEmptyString(const YAML::Node& node, const std::string& context) {
    std::string value = parseScalar<std::string>(node, context);
    if (value.empty()) {
        throw std::invalid_argument(describe(node, context) + " must not be empty");
    }
    return value;
}

double parseFiniteDouble(const YAML::Node& node, const std::string& context) {
    const double value = parseScalar<double>(node, context);
    if (!std::isfinite(value)) {
        throw std::invalid_argument(describe(node, context) + " must be finite");
    }
    return value;
}

std::size_t parsePositiveSize(const YAML::Node& node, const std::string& context) {
    const std::int64_t value = parseScalar<std::int64_t>(node, context);
    if (value <= 0) {
        throw std::invalid_argument(describe(node, context) + " must be greater than zero");
    }
    if (static_cast<std::uint64_t>(value) > std::numeric_limits<std::size_t>::max()) {
        throw std::invalid_argument(describe(node, context) + " is too large");
    }
    return static_cast<std::size_t>(value);
}

std::uint32_t parseSeed(const YAML::Node& node, const std::string& context) {
    const std::int64_t value = parseScalar<std::int64_t>(node, context);
    if (value < 0 ||
        static_cast<std::uint64_t>(value) > std::numeric_limits<std::uint32_t>::max()) {
        throw std::invalid_argument(describe(node, context) + " must fit uint32");
    }
    return static_cast<std::uint32_t>(value);
}

std::vector<double>
parseFixedVector(const YAML::Node& node, std::size_t expected_size, const std::string& context) {
    if (!node.IsSequence() || node.size() != expected_size) {
        throw std::invalid_argument(describe(node, context) + " must be a sequence of " +
                                    std::to_string(expected_size) + " finite numbers");
    }
    std::vector<double> values;
    values.reserve(expected_size);
    for (std::size_t index = 0U; index < expected_size; ++index) {
        values.push_back(
            parseFiniteDouble(node[index], context + "[" + std::to_string(index) + "]"));
    }
    return values;
}

std::filesystem::path resolvePath(const std::filesystem::path& config_directory,
                                  const YAML::Node& node,
                                  const std::string& context,
                                  const std::string& required_extension) {
    std::filesystem::path path(parseNonEmptyString(node, context));
    if (path.is_relative()) {
        path = config_directory / path;
    }
    path = path.lexically_normal();

    std::string extension = path.extension().string();
    std::transform(
        extension.begin(), extension.end(), extension.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    if (extension != required_extension) {
        throw std::invalid_argument(describe(node, context) + " must use the '" +
                                    required_extension + "' extension");
    }
    return path;
}

Eigen::Isometry3d parseTransform(const YAML::Node& node, const std::string& context) {
    if (!node.IsMap()) {
        throw std::invalid_argument(describe(node, context) + " must be a map");
    }
    const auto translation = parseFixedVector(
        requireNode(node, "translation_m", context), 3U, context + ".translation_m");
    const auto quaternion_xyzw = parseFixedVector(
        requireNode(node, "quaternion_xyzw", context), 4U, context + ".quaternion_xyzw");

    Eigen::Quaterniond quaternion(
        quaternion_xyzw[3], quaternion_xyzw[0], quaternion_xyzw[1], quaternion_xyzw[2]);
    const double norm = quaternion.norm();
    if (!std::isfinite(norm) || norm < 1.0e-12) {
        throw std::invalid_argument(describe(node, context) + " quaternion has zero norm");
    }
    if (std::abs(norm - 1.0) > kQuaternionNormTolerance) {
        throw std::invalid_argument(describe(node, context) +
                                    " quaternion norm must be within 0.001 of one");
    }
    quaternion.normalize();

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    transform.linear() = quaternion.toRotationMatrix();
    return transform;
}

PcdEncoding parseEncoding(const YAML::Node& node, const std::string& context) {
    const std::string encoding = parseNonEmptyString(node, context);
    if (encoding == "binary") {
        return PcdEncoding::kBinary;
    }
    if (encoding == "ascii") {
        return PcdEncoding::kAscii;
    }
    throw std::invalid_argument(describe(node, context) + " must be 'binary' or 'ascii'");
}

void validateEnabledInput(const ModelBuildEntry& entry) {
    std::error_code error;
    if (!std::filesystem::is_regular_file(entry.input_ply, error) || error) {
        throw std::runtime_error(
            "model '" + entry.name +
            "' input PLY is not a readable regular file: " + entry.input_ply.string());
    }
    if (entry.input_ply == entry.output_pcd) {
        throw std::invalid_argument("model '" + entry.name +
                                    "' input and output paths must differ");
    }
}

} // namespace

ModelBuildConfig loadModelBuildConfig(const std::filesystem::path& config_path) {
    if (config_path.empty()) {
        throw std::invalid_argument("config path must not be empty");
    }

    std::error_code error;
    const std::filesystem::path absolute_config =
        std::filesystem::absolute(config_path, error).lexically_normal();
    if (error || !std::filesystem::is_regular_file(absolute_config)) {
        throw std::runtime_error("model config is not a readable regular file: " +
                                 config_path.string());
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(absolute_config.string());
    } catch (const YAML::Exception& yaml_error) {
        throw std::runtime_error("failed to parse model config '" + absolute_config.string() +
                                 "': " + yaml_error.what());
    }
    if (!root.IsMap()) {
        throw std::invalid_argument("model config root must be a map");
    }

    const int schema_version =
        parseScalar<int>(requireNode(root, "schema_version", "config"), "schema_version");
    if (schema_version != kSchemaVersion) {
        throw std::invalid_argument("unsupported schema_version " + std::to_string(schema_version) +
                                    "; expected " + std::to_string(kSchemaVersion));
    }

    const YAML::Node models = requireNode(root, "models", "config");
    if (!models.IsSequence() || models.size() == 0U) {
        throw std::invalid_argument("models must be a non-empty sequence");
    }

    ModelBuildConfig config;
    config.config_path = absolute_config;
    config.config_directory = absolute_config.parent_path();
    config.models.reserve(models.size());

    std::set<std::string> names;
    std::set<std::filesystem::path> output_paths;
    for (std::size_t index = 0U; index < models.size(); ++index) {
        const YAML::Node node = models[index];
        const std::string context = "models[" + std::to_string(index) + "]";
        if (!node.IsMap()) {
            throw std::invalid_argument(describe(node, context) + " must be a map");
        }

        ModelBuildEntry entry;
        entry.name = parseNonEmptyString(requireNode(node, "name", context), context + ".name");
        if (!names.insert(entry.name).second) {
            throw std::invalid_argument("duplicate model name: " + entry.name);
        }
        entry.role = parseNonEmptyString(requireNode(node, "role", context), context + ".role");
        entry.input_ply = resolvePath(config.config_directory,
                                      requireNode(node, "input_ply", context),
                                      context + ".input_ply",
                                      ".ply");
        entry.output_pcd = resolvePath(config.config_directory,
                                       requireNode(node, "output_pcd", context),
                                       context + ".output_pcd",
                                       ".pcd");
        if (!output_paths.insert(entry.output_pcd).second) {
            throw std::invalid_argument("multiple models use output path: " +
                                        entry.output_pcd.string());
        }

        entry.sampling.input_scale_to_m = parseFiniteDouble(
            requireNode(node, "input_scale_to_m", context), context + ".input_scale_to_m");
        if (entry.sampling.input_scale_to_m <= 0.0) {
            throw std::invalid_argument(context + ".input_scale_to_m must be greater than zero");
        }
        entry.sampling.sample_count = parsePositiveSize(requireNode(node, "sample_count", context),
                                                        context + ".sample_count");
        entry.sampling.seed =
            parseSeed(requireNode(node, "random_seed", context), context + ".random_seed");
        entry.sampling.voxel_leaf_size_m = parseFiniteDouble(
            requireNode(node, "voxel_leaf_m", context), context + ".voxel_leaf_m");
        if (entry.sampling.voxel_leaf_size_m < 0.0) {
            throw std::invalid_argument(context + ".voxel_leaf_m must be non-negative");
        }
        entry.sampling.t_template_mesh = parseTransform(
            requireNode(node, "t_template_mesh", context), context + ".t_template_mesh");
        entry.pcd_encoding =
            parseEncoding(requireNode(node, "pcd_encoding", context), context + ".pcd_encoding");
        entry.overwrite =
            parseScalar<bool>(requireNode(node, "overwrite", context), context + ".overwrite");
        entry.enabled =
            parseScalar<bool>(requireNode(node, "enabled", context), context + ".enabled");

        if (entry.enabled) {
            validateEnabledInput(entry);
        }
        config.models.push_back(std::move(entry));
    }
    return config;
}

ModelBuildReport buildModels(const ModelBuildConfig& config) {
    if (config.models.empty()) {
        throw std::invalid_argument("model build config contains no models");
    }

    // Preflight every enabled output before sampling so an overwrite-policy error
    // cannot leave only a prefix of the model list regenerated.
    for (const auto& entry : config.models) {
        if (!entry.enabled) {
            continue;
        }
        validateEnabledInput(entry);
        std::error_code error;
        const bool output_exists = std::filesystem::exists(entry.output_pcd, error);
        if (error) {
            throw std::runtime_error("cannot inspect output path for model '" + entry.name +
                                     "': " + error.message());
        }
        if (output_exists && !entry.overwrite) {
            throw std::runtime_error(
                "model '" + entry.name +
                "' output already exists and overwrite is false: " + entry.output_pcd.string());
        }
        if (output_exists && !std::filesystem::is_regular_file(entry.output_pcd)) {
            throw std::runtime_error(
                "model '" + entry.name +
                "' output path is not a regular file: " + entry.output_pcd.string());
        }
    }

    ModelBuildReport report;
    report.models.reserve(config.models.size());
    for (const auto& entry : config.models) {
        ModelBuildRecord record;
        record.name = entry.name;
        record.role = entry.role;
        record.output_pcd = entry.output_pcd;
        if (!entry.enabled) {
            record.skipped = true;
            ++report.skipped_count;
            report.models.push_back(std::move(record));
            continue;
        }

        std::error_code error;
        const auto output_directory = entry.output_pcd.parent_path();
        if (!output_directory.empty()) {
            std::filesystem::create_directories(output_directory, error);
            if (error) {
                throw std::runtime_error("failed to create output directory for model '" +
                                         entry.name + "': " + error.message());
            }
        }

        const ModelSamplingResult sampled = samplePlyMesh(entry.input_ply.string(), entry.sampling);
        savePcd(
            entry.output_pcd.string(), *sampled.cloud, entry.pcd_encoding == PcdEncoding::kBinary);
        record.stats = sampled.stats;
        ++report.built_count;
        report.models.push_back(std::move(record));
    }
    return report;
}

} // namespace dart_vision::lidar
