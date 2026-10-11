# YAML 配置参考

## 适用范围

本文描述 `application/slam_options.*` 当前实际消费的 bag-only 2D 配置。完整可运行样例是
`config/iilabs3d_offline.yaml`。解析器不支持 include、表达式、环境变量替换或 Lua。

## 顶层结构

```yaml
map_builder:       # 系统和后端
trajectory_builder: # 单条活动轨迹的前端
tracking_frame: eve/base_footprint
```

`tracking_frame` 必须与 calibration 中同名字段一致。frame 会去掉开头的 `/`，除此之外
不做别名解析。

## `map_builder`

| 参数 | 类型 | 基线 | 作用 |
|---|---:|---:|---|
| `use_trajectory_builder_2d` | bool | `true` | 必须启用 2D 前端 |
| `collate_by_trajectory` | bool | `false` | 保留配置语义；当前单轨迹输入不产生差异 |

后端固定使用一个私有 FIFO worker，不提供候选约束线程数配置。PGO 的 Ceres 内部线程数
仍由 `optimization_problem.ceres_solver_options.num_threads` 独立控制。

## `map_builder.pose_graph`

| 参数 | 基线 | 作用与权衡 |
|---|---:|---|
| `optimize_every_n_nodes` | 20 | 每新增多少 node 触发一次 PGO；越小修正越及时但 CPU 越高 |
| `matcher_translation_weight` | 500 | 局部 SLAM 平移约束权重 |
| `matcher_rotation_weight` | 1600 | 局部 SLAM 旋转约束权重 |
| `max_num_final_iterations` | 200 | 轨迹结束时最大优化迭代数 |
| `global_sampling_ratio` | 0.003 | 全局定位约束采样比例 |
| `log_residual_histograms` | true | 输出优化残差分布 |

把 `optimize_every_n_nodes` 设为较小值会频繁形成同步屏障。离线吞吐优先时应结合回环密度
和最终优化评估，不要只追求更频繁的 PGO。

## `pose_graph.constraint_builder`

| 参数 | 基线 | 说明 |
|---|---:|---|
| `sampling_ratio` | 0.15 | 普通 node-submap 候选采样比例 |
| `max_constraint_distance` | 15.0 m | 局部候选最大距离 |
| `min_score` | 0.65 | 普通相关匹配接受阈值 |
| `global_localization_min_score` | 0.7 | 全局候选接受阈值 |
| `loop_closure_translation_weight` | 11000 | 回环平移残差权重 |
| `loop_closure_rotation_weight` | 100000 | 回环旋转残差权重 |
| `log_matches` | true | 输出接受/拒绝日志 |

降低 score 阈值会同时增加真回环和假回环，不能仅依据“回环数量更多”判断效果。权重决定
被接受约束进入 PGO 后的影响，不会提高粗匹配的接受概率。

### `fast_correlative_scan_matcher`

| 参数 | 基线 | 说明 |
|---|---:|---|
| `linear_search_window` | 7.0 m | 回环平移搜索半径 |
| `angular_search_window` | 0.5236 rad | 回环角度搜索半径 |
| `branch_and_bound_depth` | 7 | 多分辨率搜索深度 |

窗口和深度主要影响 constraint worker CPU。搜索索引按 submap 构建一次，多组候选共享。

### 后端 `ceres_scan_matcher`

用于精化已通过快速相关搜索的回环相对位姿。`occupied_space_weight`、
`translation_weight`、`rotation_weight` 分别控制栅格占据残差和初值先验。其
`ceres_solver_options.num_threads` 基线为 1，避免大量并行候选内部再次展开线程。

## `pose_graph.optimization_problem`

| 参数 | 基线 | 说明 |
|---|---:|---|
| `huber_scale` | 10 | 约束鲁棒损失尺度 |
| `local_slam_pose_translation_weight` | 100000 | 连续局部位姿平移权重 |
| `local_slam_pose_rotation_weight` | 100000 | 连续局部位姿旋转权重 |
| `odometry_translation_weight` | 100 | odometry 平移权重 |
| `odometry_rotation_weight` | 10 | odometry 旋转权重 |
| `log_solver_summary` | false | 输出 Ceres 完整摘要 |

PGO 的 `ceres_solver_options.max_num_iterations` 基线为 50，`num_threads` 为 7。它只控制
单后端 worker 调用 Ceres 后的数值求解并行度，不会并行执行多个 constraint 候选。

## `trajectory_builder.trajectory_builder_2d`

### 输入裁剪与累积

| 参数 | 基线 | 说明 |
|---|---:|---|
| `min_range` | 0.05 m | 小于此距离的 return 丢弃 |
| `max_range` | 30.0 m | 超出此距离按 missing ray 处理 |
| `min_z` / `max_z` | -0.8 / 2.0 m | tracking frame 中高度裁剪 |
| `missing_data_ray_length` | 15.0 m | 无回波 ray 的自由空间长度 |
| `num_accumulated_range_data` | 1 | 累积多少帧后匹配一次 |
| `voxel_filter_size` | 0.025 m | 固定体素滤波尺寸 |

严格 2D 仍保留 z 裁剪，因为雷达外参可能带 z 和轻微倾角；算法最终优化 x/y/yaw。

### 自适应体素滤波

普通匹配和回环各有一组 `max_length`、`min_num_points`、`max_range`。滤波器在点数足够的
前提下尽量使用较大体素；回环通常允许更粗的点云以降低候选计算量。

### 前端 scan matching

可选 `use_online_correlative_scan_matching`（默认 `false`）和
`real_time_correlative_scan_matcher`。开启后，Ceres 之前先在预测位姿附近做穷举
相关搜索，用来把 yaw 从里程计先验上拉开。窗口和偏离代价：

- `linear_search_window` / `angular_search_window`：平移、旋转搜索半径；
- `translation_delta_cost_weight` / `rotation_delta_cost_weight`：偏离预测的指数惩罚。

前端 `ceres_scan_matcher`：

- `occupied_space_weight`：scan 与概率栅格一致性；
- `translation_weight`：偏离预测平移的代价；
- `rotation_weight`：偏离预测 yaw 的代价；
- solver 基线 20 次迭代、1 线程。

相关搜索开启时，Ceres 的初值是相关匹配结果；平移先验仍指向里程计预测。先验过紧
会把解拉回轮式里程计。

### `motion_filter`

只有时间、距离、角度三个变化都低于门限时才丢弃 node。基线是 5 秒、0.06 米、3 度。
门限增大可减少 node 和后端负载，但会降低 submap 约束密度。

### `skip_insert_*`（仅真机 LD14）

这些字段默认 `0`（未写出即关闭）。只应出现在 `config/record_offline.yaml`：

| 参数 | record 值 | 说明 |
|---|---:|---|
| `skip_insert_after_gap_seconds` | 0.25（机端 live 常用 0.5） | 相邻有效雷达间隔超过该值时仍匹配，但不写入占据栅格 |
| `skip_insert_while_turning_radians` | 0.06 | 本圈相对上一帧的轮速或匹配转角超过该值时不写入 |
| `skip_insert_destort_yaw_radians` | 0.08 | 本圈首束到末束的里程计航向差超过该值时不写入（帧内畸变） |
| `skip_insert_peak_segment_seconds` | 0.08 | 帧内滑动窗口长度；与下行一起用于峰值段航向门控 |
| `skip_insert_peak_segment_yaw_radians` | 0.05 | 任一窗口内里程计 \|yaw\| 超过该值时不写入（局部急转/部分点云畸变） |
| `skip_insert_half_scan_arm_yaw_radians` | 0.06 | 任一帧间/帧内航向信号超过该值时武装半圈检查 |
| `skip_insert_half_scan_after_turn_seconds` | 1.0 | 武装保持时间；仅在此窗口内跑半圈匹配 |
| `skip_insert_half_scan_linear_search_window` | 0.09 | 半圈专用 correlative 平移搜索窗（m） |
| `skip_insert_half_scan_angular_search_window` | 0.05 | 半圈专用 correlative 航向搜索窗（rad） |
| `skip_insert_half_scan_yaw_radians` | 0.03 | early/late 半圈位姿航向差超过该值时不写入 |
| `skip_insert_half_scan_translation_meters` | 0.06 | early/late 半圈位姿平移差超过该值时不写入（与航向为 OR） |
| `skip_insert_max_range_meters` | 5.0 | 仅插图用的最远距离；更远回波不写入（`0` 关闭）。匹配仍用 `max_range` |
| `skip_insert_far_disagree_min_range_meters` | 1.5 | 仅对距离 ≥ 该值的回波做射线残差滤除；近距仍全部写入 |
| `skip_insert_far_disagree_base_meters` | 0.04 | 与射线上**第一堵墙**的允许距离差基数；`≤0` 关闭该滤除 |
| `skip_insert_far_disagree_per_meter` | 0.01 | 允许残差随距离线性放宽：`base + per_meter · R` |
| `skip_insert_far_disagree_ratio` | 0.22 | 远点中鬼影拒绝占比 ≥ 该值时整帧不插图（`0` 关闭；不含超插图最远的点） |
| `tracking_min_match_agree_ratio` | 0.35 | 匹配后中距回波落在占据邻域的比例低于该值视为差匹配（`0` 关闭） |
| `tracking_min_correlative_score` | 0 | correlative 分数门槛；仅当一致率也偏低时参与判 LOST（`0` 关闭） |
| `tracking_bad_match_frames` | 3 | 连续差匹配帧数达到后进入 LOST，触发全局 `Relocalize` |

差匹配时先用轮速预测、不插图；连续超限则保持末位姿并走已有重定位。插图滤除只挡写图，不负责把错位姿拉回。

目标是减少多层墙**写入**地图：点级丢掉鬼影带回波，帧级在弯扫上禁止整帧落图。匹配仍用完整点云；调试琥珀色仅供对照。

仿真 / `datasets/exploration` / `iilabs3d_offline.yaml` 不得填写。Gazebo 的
`time_increment == 0`，旋转去畸变不会启用；跳过写入会让探索原地转丢图。

### `pose_extrapolator.constant_velocity`

| 参数 | 基线 | 当前含义 |
|---|---:|---|
| `pose_queue_duration` | 0.001 s | 估计速度使用的最短 pose 历史窗口 |

当前没有外部 IMU 输入。产品假设严格二维运动，roll/pitch 不从真实惯导观测恢复；未来若
里程计提供 IMU 姿态，应先定义新的输入与标定契约，不能把角速度伪装成现有 IMU。

### `submaps`

| 参数 | 基线 | 说明 |
|---|---:|---|
| `num_range_data` | 35 | 单个 submap 接收的 range data 数量尺度 |
| `grid_options_2d.grid_type` | `PROBABILITY_GRID` | 当前唯一支持网格 |
| `grid_options_2d.resolution` | 0.03 m | 栅格分辨率 |
| `insert_free_space` | true | 是否写入 miss ray |
| `hit_probability` | 0.55 | 命中单元更新概率 |
| `miss_probability` | 0.49 | 穿越单元更新概率 |

分辨率越小，内存、插入和匹配成本越高。`hit_probability` 应大于 0.5，
`miss_probability` 应小于 0.5；极端值会让地图过快饱和。

## 调参顺序

1. 先固定并验证标定、时间偏移和量程；
2. 再调前端滤波、预测和 scan matcher；
3. 再调 submap 尺寸与分辨率；
4. 最后调回环阈值、采样率和 PGO 权重；
5. 每一步同时观察完整轨迹和短窗口，不跨多组参数建立无法归因的新基线。

## 配置错误策略

`ParameterDictionary` 会跟踪字段消费。配置文件不是“可带多余字段的模板”：拼写错误、
上游遗留选项或未支持功能字段可能直接失败。这一策略用于防止用户以为某个参数生效，
实际却被程序忽略。
