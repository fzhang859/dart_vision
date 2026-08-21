# dart_vision_lidar_model

Mid-70 LiDAR 模板的离线生成包。它读取普通 YAML 文件，将任意数量的 PLY
三角网格按面积加权均匀采样、转换到模板坐标系、可选 VoxelGrid 后保存为 PCD。
在线定位不依赖 yaml-cpp，也不会在启动时自动重建模型。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dart_vision_lidar_model --symlink-install
source install/setup.bash
```

## 批量生成

复制并修改
[`config/model_build_example.yaml`](config/model_build_example.yaml)，确认每个模型的
CAD 单位、角色和坐标变换后，将对应 `enabled` 改为 `true`：

```bash
ros2 run dart_vision_lidar_model dart_vision_model_builder \
  --config /absolute/path/mid70_models.yaml
```

这是普通 yaml-cpp 配置，不是 ROS 参数文件。`input_ply` 和 `output_pcd` 的相对路径
均相对于 YAML 所在目录解析。顶层格式为：

```yaml
schema_version: 1
models:
  - name: moving_armor
    role: armor
    input_ply: raw/armor.ply
    output_pcd: processed/armor.pcd
    input_scale_to_m: 0.001
    sample_count: 60000
    random_seed: 7003
    voxel_leaf_m: 0.003
    t_template_mesh:
      translation_m: [0.0, 0.0, 0.0]
      quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]
    pcd_encoding: binary
    overwrite: false
    enabled: true
```

变换方向明确为：

```text
p_template_m = t_template_mesh * (input_scale_to_m * p_mesh)
```

`translation_m` 以米为单位，四元数顺序为 `xyzw` 且模长必须在 1 的 0.001
范围内。`voxel_leaf_m: 0` 关闭降采样。`pcd_encoding` 仅接受 `binary` 或
`ascii`。模型名和输出路径必须唯一；启用模型的输入必须是 `.ply` 普通文件；
输出必须是 `.pcd`。输出目录会自动创建，已有输出只有在 `overwrite: true` 时才会
覆盖。任意校验、读取、采样或保存失败都会使进程返回非零。

`enabled: false` 的条目仅记录为跳过，其输入文件可以暂时不存在，适合尚未确认的
CAD 模型。批处理在开始采样前先检查所有启用模型的输出覆盖策略，但处理过程中发生
错误时不会回滚此前已写出的 PCD。
