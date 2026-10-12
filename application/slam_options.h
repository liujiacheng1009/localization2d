// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef CARTOGRAPHER_MAPPING_OPTIONS_H_
#define CARTOGRAPHER_MAPPING_OPTIONS_H_

#include <optional>
#include <string>

#include "cartographer/application/config.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/foundation/time.h"
#include "cartographer/foundation/transform.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {

#define CARTOGRAPHER_OPTION_FIELD(Type, name) \
 private:                                      \
  Type name##_;                                \
                                               \
 public:                                       \
  const Type& name() const { return name##_; } \
  void set_##name(const Type& value) { name##_ = value; }

#define CARTOGRAPHER_OPTION_SCALAR(Type, name) \
 private:                                       \
  Type name##_{};                               \
                                                \
 public:                                        \
  Type name() const { return name##_; }          \
  void set_##name(Type value) { name##_ = value; }

struct CeresSolverOptions {
  CARTOGRAPHER_OPTION_SCALAR(bool, use_nonmonotonic_steps)
  CARTOGRAPHER_OPTION_SCALAR(int, max_num_iterations)
  CARTOGRAPHER_OPTION_SCALAR(int, num_threads)
};

struct AdaptiveVoxelFilterOptions {
  CARTOGRAPHER_OPTION_SCALAR(float, max_length)
  CARTOGRAPHER_OPTION_SCALAR(float, min_num_points)
  CARTOGRAPHER_OPTION_SCALAR(float, max_range)
};

struct MotionFilterOptions {
  CARTOGRAPHER_OPTION_SCALAR(double, max_time_seconds)
  CARTOGRAPHER_OPTION_SCALAR(double, max_distance_meters)
  CARTOGRAPHER_OPTION_SCALAR(double, max_angle_radians)
};

struct CeresScanMatcherOptions2D {
  CARTOGRAPHER_OPTION_SCALAR(double, occupied_space_weight)
  CARTOGRAPHER_OPTION_SCALAR(double, translation_weight)
  CARTOGRAPHER_OPTION_SCALAR(double, rotation_weight)
  CARTOGRAPHER_OPTION_FIELD(CeresSolverOptions, ceres_solver_options)
  CeresSolverOptions* mutable_ceres_solver_options() {
    return &ceres_solver_options_;
  }
};

struct FastCorrelativeScanMatcherOptions2D {
  CARTOGRAPHER_OPTION_SCALAR(double, linear_search_window)
  CARTOGRAPHER_OPTION_SCALAR(double, angular_search_window)
  CARTOGRAPHER_OPTION_SCALAR(int, branch_and_bound_depth)
};

struct RealTimeCorrelativeScanMatcherOptions2D {
  CARTOGRAPHER_OPTION_SCALAR(double, linear_search_window)
  CARTOGRAPHER_OPTION_SCALAR(double, angular_search_window)
  CARTOGRAPHER_OPTION_SCALAR(double, translation_delta_cost_weight)
  CARTOGRAPHER_OPTION_SCALAR(double, rotation_delta_cost_weight)
};

struct ProbabilityGridRangeDataInserterOptions2D {
  CARTOGRAPHER_OPTION_SCALAR(double, hit_probability)
  CARTOGRAPHER_OPTION_SCALAR(double, miss_probability)
  CARTOGRAPHER_OPTION_SCALAR(bool, insert_free_space)
};

struct GridOptions2D {
  enum GridType { INVALID_GRID = 0, PROBABILITY_GRID = 1 };
  CARTOGRAPHER_OPTION_SCALAR(GridType, grid_type)
  CARTOGRAPHER_OPTION_SCALAR(float, resolution)
};

struct RangeDataInserterOptions {
  enum RangeDataInserterType {
    INVALID_INSERTER = 0,
    PROBABILITY_GRID_INSERTER_2D = 1
  };
  CARTOGRAPHER_OPTION_SCALAR(RangeDataInserterType, range_data_inserter_type)
  CARTOGRAPHER_OPTION_FIELD(ProbabilityGridRangeDataInserterOptions2D,
                            probability_grid_range_data_inserter_options_2d)
  ProbabilityGridRangeDataInserterOptions2D*
  mutable_probability_grid_range_data_inserter_options_2d() {
    return &probability_grid_range_data_inserter_options_2d_;
  }
};

struct SubmapsOptions2D {
  CARTOGRAPHER_OPTION_SCALAR(int, num_range_data)
  CARTOGRAPHER_OPTION_FIELD(GridOptions2D, grid_options_2d)
  CARTOGRAPHER_OPTION_FIELD(RangeDataInserterOptions,
                            range_data_inserter_options)
  GridOptions2D* mutable_grid_options_2d() { return &grid_options_2d_; }
  RangeDataInserterOptions* mutable_range_data_inserter_options() {
    return &range_data_inserter_options_;
  }
};

struct ConstantVelocityPoseExtrapolatorOptions {
  CARTOGRAPHER_OPTION_SCALAR(double, pose_queue_duration)
};

struct PoseExtrapolatorOptions {
  CARTOGRAPHER_OPTION_FIELD(ConstantVelocityPoseExtrapolatorOptions,
                            constant_velocity)
  ConstantVelocityPoseExtrapolatorOptions* mutable_constant_velocity() {
    return &constant_velocity_;
  }
};

struct LocalTrajectoryBuilderOptions2D {
  CARTOGRAPHER_OPTION_SCALAR(float, min_range)
  CARTOGRAPHER_OPTION_SCALAR(float, max_range)
  CARTOGRAPHER_OPTION_SCALAR(float, min_z)
  CARTOGRAPHER_OPTION_SCALAR(float, max_z)
  CARTOGRAPHER_OPTION_SCALAR(float, missing_data_ray_length)
  CARTOGRAPHER_OPTION_SCALAR(int, num_accumulated_range_data)
  CARTOGRAPHER_OPTION_SCALAR(float, voxel_filter_size)
  CARTOGRAPHER_OPTION_FIELD(AdaptiveVoxelFilterOptions,
                            adaptive_voxel_filter_options)
  CARTOGRAPHER_OPTION_FIELD(AdaptiveVoxelFilterOptions,
                            loop_closure_adaptive_voxel_filter_options)
  CARTOGRAPHER_OPTION_SCALAR(bool, use_online_correlative_scan_matching)
  CARTOGRAPHER_OPTION_FIELD(RealTimeCorrelativeScanMatcherOptions2D,
                            real_time_correlative_scan_matcher_options)
  CARTOGRAPHER_OPTION_FIELD(CeresScanMatcherOptions2D,
                            ceres_scan_matcher_options)
  CARTOGRAPHER_OPTION_FIELD(MotionFilterOptions, motion_filter_options)
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_after_gap_seconds)
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_while_turning_radians)
  // Absolute intra-scan yaw from first to last beam (odometry deskew).
  // 0 disables. LD14 default is 0.08 rad when set in record_offline.yaml.
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_destort_yaw_radians)
  // Peak |yaw| of odometry over any intra-scan window of this length. Catches
  // a fast turn that only bends part of the sweep when first→last destort is
  // still small. 0 window or 0 yaw limit disables.
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_peak_segment_seconds)
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_peak_segment_yaw_radians)
  // After the full-scan match, match the early/late temporal halves of the
  // sweep independently. Skip insert when the two half poses disagree by more
  // than these limits (0 disables). Intended only in a post-turn window.
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_half_scan_yaw_radians)
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_half_scan_translation_meters)
  // Soft-arm half-scan when odom/matched/destort/peak yaw exceeds this; keep
  // armed for this many seconds after the last arming signal. 0 disables the
  // half-scan gate entirely (never runs).
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_half_scan_arm_yaw_radians)
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_half_scan_after_turn_seconds)
  // Tight correlative window for half-scan only (cheaper than the online
  // matcher). 0 falls back to the online real_time_correlative windows.
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_half_scan_linear_search_window)
  CARTOGRAPHER_OPTION_SCALAR(double,
                             skip_insert_half_scan_angular_search_window)
  // Insert-only max range. Matching still uses trajectory max_range. 0 = off.
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_max_range_meters)
  // Insert-only: drop returns beyond min_range that disagree with the active
  // submap. Threshold is base + per_meter * range. Matching still uses the
  // full cloud. base <= 0 disables.
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_far_disagree_min_range_meters)
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_far_disagree_base_meters)
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_far_disagree_per_meter)
  // If rejected_far / far_returns >= this, skip the whole insert (bent sweep
  // would still paint near hits into a multilayer wall). 0 disables.
  CARTOGRAPHER_OPTION_SCALAR(double, skip_insert_far_disagree_ratio)
  // Tracking quality: fraction of mid-range returns that land near occupied
  // cells after scan match. Below this for tracking_bad_match_frames → LOST
  // and global Relocalize. 0 disables.
  CARTOGRAPHER_OPTION_SCALAR(double, tracking_min_match_agree_ratio)
  // Optional correlative score floor (online correlative only). 0 disables.
  CARTOGRAPHER_OPTION_SCALAR(double, tracking_min_correlative_score)
  // Consecutive bad-match frames before latching LOST / holding pose.
  CARTOGRAPHER_OPTION_SCALAR(int, tracking_bad_match_frames)
  CARTOGRAPHER_OPTION_FIELD(PoseExtrapolatorOptions,
                            pose_extrapolator_options)
  CARTOGRAPHER_OPTION_FIELD(SubmapsOptions2D, submaps_options)
  AdaptiveVoxelFilterOptions* mutable_adaptive_voxel_filter_options() {
    return &adaptive_voxel_filter_options_;
  }
  AdaptiveVoxelFilterOptions*
  mutable_loop_closure_adaptive_voxel_filter_options() {
    return &loop_closure_adaptive_voxel_filter_options_;
  }
  RealTimeCorrelativeScanMatcherOptions2D*
  mutable_real_time_correlative_scan_matcher_options() {
    return &real_time_correlative_scan_matcher_options_;
  }
  CeresScanMatcherOptions2D* mutable_ceres_scan_matcher_options() {
    return &ceres_scan_matcher_options_;
  }
  MotionFilterOptions* mutable_motion_filter_options() {
    return &motion_filter_options_;
  }
  PoseExtrapolatorOptions* mutable_pose_extrapolator_options() {
    return &pose_extrapolator_options_;
  }
  SubmapsOptions2D* mutable_submaps_options() { return &submaps_options_; }
};

struct Isam2OptimizerOptions {
  // ISAM2 relinearize knobs; <=0 uses library defaults (0.1 / 10).
  CARTOGRAPHER_OPTION_SCALAR(double, relinearize_threshold)
  CARTOGRAPHER_OPTION_SCALAR(int, relinearize_skip)
};

struct OptimizationProblemOptions {
  CARTOGRAPHER_OPTION_SCALAR(double, huber_scale)
  CARTOGRAPHER_OPTION_SCALAR(double, local_slam_pose_translation_weight)
  CARTOGRAPHER_OPTION_SCALAR(double, local_slam_pose_rotation_weight)
  CARTOGRAPHER_OPTION_SCALAR(double, odometry_translation_weight)
  CARTOGRAPHER_OPTION_SCALAR(double, odometry_rotation_weight)
  CARTOGRAPHER_OPTION_SCALAR(bool, log_solver_summary)
  CARTOGRAPHER_OPTION_FIELD(CeresSolverOptions, ceres_solver_options)
  CARTOGRAPHER_OPTION_FIELD(Isam2OptimizerOptions, isam2_options)
  CeresSolverOptions* mutable_ceres_solver_options() {
    return &ceres_solver_options_;
  }
  Isam2OptimizerOptions* mutable_isam2_options() { return &isam2_options_; }
};

struct ConstraintBuilderOptions {
  CARTOGRAPHER_OPTION_SCALAR(double, sampling_ratio)
  CARTOGRAPHER_OPTION_SCALAR(double, max_constraint_distance)
  CARTOGRAPHER_OPTION_SCALAR(double, min_score)
  CARTOGRAPHER_OPTION_SCALAR(double, global_localization_min_score)
  CARTOGRAPHER_OPTION_SCALAR(double, loop_closure_translation_weight)
  CARTOGRAPHER_OPTION_SCALAR(double, loop_closure_rotation_weight)
  CARTOGRAPHER_OPTION_SCALAR(bool, log_matches)
  CARTOGRAPHER_OPTION_FIELD(FastCorrelativeScanMatcherOptions2D,
                            fast_correlative_scan_matcher_options)
  CARTOGRAPHER_OPTION_FIELD(CeresScanMatcherOptions2D,
                            ceres_scan_matcher_options)
  FastCorrelativeScanMatcherOptions2D*
  mutable_fast_correlative_scan_matcher_options() {
    return &fast_correlative_scan_matcher_options_;
  }
  CeresScanMatcherOptions2D* mutable_ceres_scan_matcher_options() {
    return &ceres_scan_matcher_options_;
  }
};

struct PoseGraphOptions {
  struct OverlappingSubmapsTrimmerOptions2D {
    CARTOGRAPHER_OPTION_SCALAR(int, fresh_submaps_count)
    CARTOGRAPHER_OPTION_SCALAR(double, min_covered_area)
    CARTOGRAPHER_OPTION_SCALAR(int, min_added_submaps_count)
  };
  CARTOGRAPHER_OPTION_SCALAR(int, optimize_every_n_nodes)
  // Pose-graph SPA: "isam2" (default) | "ceres".
  CARTOGRAPHER_OPTION_FIELD(std::string, optimizer)
  CARTOGRAPHER_OPTION_SCALAR(double, matcher_translation_weight)
  CARTOGRAPHER_OPTION_SCALAR(double, matcher_rotation_weight)
  CARTOGRAPHER_OPTION_SCALAR(int, max_num_final_iterations)
  CARTOGRAPHER_OPTION_SCALAR(double, global_sampling_ratio)
  CARTOGRAPHER_OPTION_SCALAR(bool, log_residual_histograms)
  CARTOGRAPHER_OPTION_FIELD(ConstraintBuilderOptions,
                            constraint_builder_options)
  CARTOGRAPHER_OPTION_FIELD(OptimizationProblemOptions,
                            optimization_problem_options)
  ConstraintBuilderOptions* mutable_constraint_builder_options() {
    return &constraint_builder_options_;
  }
  OptimizationProblemOptions* mutable_optimization_problem_options() {
    return &optimization_problem_options_;
  }
  OverlappingSubmapsTrimmerOptions2D*
  mutable_overlapping_submaps_trimmer_2d() {
    return &overlapping_submaps_trimmer_2d_.emplace();
  }
  bool has_overlapping_submaps_trimmer_2d() const {
    return overlapping_submaps_trimmer_2d_.has_value();
  }
  const OverlappingSubmapsTrimmerOptions2D&
  overlapping_submaps_trimmer_2d() const {
    return overlapping_submaps_trimmer_2d_.value();
  }

 private:
  std::optional<OverlappingSubmapsTrimmerOptions2D>
      overlapping_submaps_trimmer_2d_;
};

struct SlamSystemOptions {
  CARTOGRAPHER_OPTION_SCALAR(bool, use_trajectory_builder_2d)
  CARTOGRAPHER_OPTION_SCALAR(bool, collate_by_trajectory)
  CARTOGRAPHER_OPTION_FIELD(PoseGraphOptions, pose_graph_options)
  PoseGraphOptions* mutable_pose_graph_options() { return &pose_graph_options_; }
};

struct InitialTrajectoryPose {
  transform::Rigid2d relative_pose;
  int to_trajectory_id = 0;
  common::Time timestamp;
};

struct TrajectoryBuilderOptions {
  struct PureLocalizationTrimmerOptions {
    CARTOGRAPHER_OPTION_SCALAR(int, max_submaps_to_keep)
  };

  CARTOGRAPHER_OPTION_SCALAR(bool, pure_localization)
  CARTOGRAPHER_OPTION_FIELD(LocalTrajectoryBuilderOptions2D,
                            trajectory_builder_2d_options)
  bool has_trajectory_builder_2d_options() const { return true; }
  LocalTrajectoryBuilderOptions2D* mutable_trajectory_builder_2d_options() {
    return &trajectory_builder_2d_options_;
  }

  bool has_initial_trajectory_pose() const {
    return initial_trajectory_pose_.has_value();
  }
  const InitialTrajectoryPose& initial_trajectory_pose() const {
    return initial_trajectory_pose_.value();
  }
  InitialTrajectoryPose* mutable_initial_trajectory_pose() {
    return &initial_trajectory_pose_.emplace();
  }

  bool has_pure_localization_trimmer() const {
    return pure_localization_trimmer_.has_value();
  }
  const PureLocalizationTrimmerOptions& pure_localization_trimmer() const {
    return pure_localization_trimmer_.value();
  }
  PureLocalizationTrimmerOptions* mutable_pure_localization_trimmer() {
    return &pure_localization_trimmer_.emplace();
  }

  bool has_pose_graph_odometry_motion_filter() const {
    return pose_graph_odometry_motion_filter_.has_value();
  }
  const MotionFilterOptions& pose_graph_odometry_motion_filter() const {
    return pose_graph_odometry_motion_filter_.value();
  }
  MotionFilterOptions* mutable_pose_graph_odometry_motion_filter() {
    return &pose_graph_odometry_motion_filter_.emplace();
  }

 private:
  std::optional<InitialTrajectoryPose> initial_trajectory_pose_;
  std::optional<PureLocalizationTrimmerOptions> pure_localization_trimmer_;
  std::optional<MotionFilterOptions> pose_graph_odometry_motion_filter_;
};

TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::ParameterDictionary* parameter_dictionary);

LocalTrajectoryBuilderOptions2D CreateLocalTrajectoryBuilderOptions2D(
    common::ParameterDictionary* parameter_dictionary);
PoseGraphOptions CreatePoseGraphOptions(
    common::ParameterDictionary* parameter_dictionary);

namespace optimization {
OptimizationProblemOptions CreateOptimizationProblemOptions(
    common::ParameterDictionary* parameter_dictionary);
}  // namespace optimization

#undef CARTOGRAPHER_OPTION_FIELD
#undef CARTOGRAPHER_OPTION_SCALAR

}  // namespace mapping

namespace common {
mapping::CeresSolverOptions CreateCeresSolverOptionsConfig(
    ParameterDictionary* parameter_dictionary);
ceres::Solver::Options CreateCeresSolverOptions(
    const mapping::CeresSolverOptions& config);
}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_OPTIONS_H_
