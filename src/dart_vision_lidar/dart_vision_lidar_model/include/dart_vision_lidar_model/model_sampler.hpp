#ifndef DART_VISION_LIDAR_MODEL_MODEL_SAMPLER_HPP
#define DART_VISION_LIDAR_MODEL_MODEL_SAMPLER_HPP

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace dart_vision::lidar {

/// Configuration for area-weighted sampling of a CAD mesh.
struct ModelSamplingOptions {
    /// Number of points generated before optional voxel downsampling.
    std::size_t sample_count{100000U};

    /// Seed used by the deterministic pseudo-random sampler.
    std::uint32_t seed{0U};

    /// Multiplier converting every input mesh coordinate to metres.
    double input_scale_to_m{1.0};

    /**
 * Rigid transform from the scaled mesh frame to the template frame:
 * p_template_m = t_template_mesh * (input_scale_to_m * p_mesh).
 */
    Eigen::Isometry3d t_template_mesh{Eigen::Isometry3d::Identity()};

    /// VoxelGrid leaf size in metres. Zero disables downsampling.
    double voxel_leaf_size_m{0.0};
};

struct ModelSamplingStats {
    std::size_t input_polygon_count{0U};
    std::size_t candidate_triangle_count{0U};
    std::size_t sampled_triangle_count{0U};
    std::size_t skipped_invalid_triangle_count{0U};
    std::size_t skipped_degenerate_triangle_count{0U};
    std::size_t point_count_before_voxel{0U};
    std::size_t point_count_after_voxel{0U};
    double surface_area_m2{0.0};
};

struct ModelSamplingResult {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    ModelSamplingStats stats;
};

/// Loads a PLY file as a PolygonMesh or throws std::runtime_error.
pcl::PolygonMesh loadPlyMesh(const std::string& ply_path);

/**
 * Triangulates polygon faces using a fan, removes invalid/degenerate
 * triangles, and samples the remaining surface in proportion to triangle
 * area. Throws std::invalid_argument for invalid options and
 * std::runtime_error when no sampleable surface remains.
 */
ModelSamplingResult sampleMesh(const pcl::PolygonMesh& mesh, const ModelSamplingOptions& options);

/// Convenience wrapper around loadPlyMesh() and sampleMesh().
ModelSamplingResult samplePlyMesh(const std::string& ply_path, const ModelSamplingOptions& options);

/// Saves XYZ points to PCD. Throws std::runtime_error on failure.
void savePcd(const std::string& pcd_path,
             const pcl::PointCloud<pcl::PointXYZ>& cloud,
             bool binary = true);

} // namespace dart_vision::lidar

#endif // DART_VISION_LIDAR_MODEL_MODEL_SAMPLER_HPP
