// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/application/slam_options.h"

#include "cartographer/foundation/voxel_filter.h"
#include "cartographer/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/frontend/motion_filter.h"
#include "cartographer/frontend/pose_extrapolator.h"
#include "cartographer/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/submap_2d.h"

namespace cartographer {
namespace common {

mapping::CeresSolverOptions CreateCeresSolverOptionsConfig(
    ParameterDictionary* parameter_dictionary) {
  mapping::CeresSolverOptions config;
  config.set_use_nonmonotonic_steps(
      parameter_dictionary->GetBool("use_nonmonotonic_steps"));
  config.set_max_num_iterations(
      parameter_dictionary->GetNonNegativeInt("max_num_iterations"));
  config.set_num_threads(parameter_dictionary->GetNonNegativeInt("num_threads"));
  CHECK_GT(config.max_num_iterations(), 0);
  CHECK_GT(config.num_threads(), 0);
  return config;
}

ceres::Solver::Options CreateCeresSolverOptions(
    const mapping::CeresSolverOptions& config) {
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = config.use_nonmonotonic_steps();
  options.max_num_iterations = config.max_num_iterations();
  options.num_threads = config.num_threads();
  return options;
}

}  // namespace common

namespace mapping {

namespace {

ConstraintBuilderOptions CreateConstraintBuilderOptions(
    common::ParameterDictionary* const dictionary) {
  ConstraintBuilderOptions options;
  options.set_sampling_ratio(dictionary->GetDouble("sampling_ratio"));
  options.set_max_constraint_distance(
      dictionary->GetDouble("max_constraint_distance"));
  options.set_min_score(dictionary->GetDouble("min_score"));
  options.set_global_localization_min_score(
      dictionary->GetDouble("global_localization_min_score"));
  options.set_loop_closure_translation_weight(
      dictionary->GetDouble("loop_closure_translation_weight"));
  options.set_loop_closure_rotation_weight(
      dictionary->GetDouble("loop_closure_rotation_weight"));
  options.set_log_matches(dictionary->GetBool("log_matches"));
  *options.mutable_fast_correlative_scan_matcher_options() =
      scan_matching::CreateFastCorrelativeScanMatcherOptions2D(
          dictionary->GetDictionary("fast_correlative_scan_matcher").get());
  *options.mutable_ceres_scan_matcher_options() =
      scan_matching::CreateCeresScanMatcherOptions2D(
          dictionary->GetDictionary("ceres_scan_matcher").get());
  return options;
}

void PopulatePureLocalizationTrimmerOptions(
    TrajectoryBuilderOptions* const options,
    common::ParameterDictionary* const dictionary) {
  constexpr char kKey[] = "pure_localization_trimmer";
  if (!dictionary->HasKey(kKey)) return;
  auto values = dictionary->GetDictionary(kKey);
  options->mutable_pure_localization_trimmer()->set_max_submaps_to_keep(
      values->GetInt("max_submaps_to_keep"));
}

void PopulatePoseGraphOdometryMotionFilterOptions(
    TrajectoryBuilderOptions* const options,
    common::ParameterDictionary* const dictionary) {
  constexpr char kKey[] = "pose_graph_odometry_motion_filter";
  if (!dictionary->HasKey(kKey)) return;
  auto values = dictionary->GetDictionary(kKey);
  auto* filter = options->mutable_pose_graph_odometry_motion_filter();
  filter->set_max_time_seconds(values->GetDouble("max_time_seconds"));
  filter->set_max_distance_meters(values->GetDouble("max_distance_meters"));
  filter->set_max_angle_radians(values->GetDouble("max_angle_radians"));
}

ConstantVelocityPoseExtrapolatorOptions
CreateConstantVelocityPoseExtrapolatorOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  ConstantVelocityPoseExtrapolatorOptions options;
  options.set_pose_queue_duration(
      parameter_dictionary->GetDouble("pose_queue_duration"));
  return options;
}

}  // namespace

TrajectoryBuilderOptions CreateTrajectoryBuilderOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  TrajectoryBuilderOptions options;
  *options.mutable_trajectory_builder_2d_options() =
      CreateLocalTrajectoryBuilderOptions2D(
          parameter_dictionary->GetDictionary("trajectory_builder_2d").get());
  PopulatePureLocalizationTrimmerOptions(&options, parameter_dictionary);
  PopulatePoseGraphOdometryMotionFilterOptions(&options,
                                                parameter_dictionary);
  return options;
}

PoseExtrapolatorOptions CreatePoseExtrapolatorOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  PoseExtrapolatorOptions options;
  *options.mutable_constant_velocity() =
      CreateConstantVelocityPoseExtrapolatorOptions(
          parameter_dictionary->GetDictionary("constant_velocity").get());
  return options;
}

namespace {

void PopulateOverlappingSubmapsTrimmerOptions2D(
    PoseGraphOptions* const pose_graph_options,
    common::ParameterDictionary* const parameter_dictionary) {
  constexpr char kDictionaryKey[] = "overlapping_submaps_trimmer_2d";
  if (!parameter_dictionary->HasKey(kDictionaryKey)) return;
  auto dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
  auto* options = pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
  options->set_fresh_submaps_count(dictionary->GetInt("fresh_submaps_count"));
  options->set_min_covered_area(dictionary->GetDouble("min_covered_area"));
  options->set_min_added_submaps_count(
      dictionary->GetInt("min_added_submaps_count"));
}

}  // namespace

PoseGraphOptions CreatePoseGraphOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  PoseGraphOptions options;
  options.set_optimize_every_n_nodes(
      parameter_dictionary->GetInt("optimize_every_n_nodes"));
  *options.mutable_constraint_builder_options() =
      CreateConstraintBuilderOptions(
          parameter_dictionary->GetDictionary("constraint_builder").get());
  options.set_matcher_translation_weight(
      parameter_dictionary->GetDouble("matcher_translation_weight"));
  options.set_matcher_rotation_weight(
      parameter_dictionary->GetDouble("matcher_rotation_weight"));
  *options.mutable_optimization_problem_options() =
      optimization::CreateOptimizationProblemOptions(
          parameter_dictionary->GetDictionary("optimization_problem").get());
  options.set_max_num_final_iterations(
      parameter_dictionary->GetNonNegativeInt("max_num_final_iterations"));
  CHECK_GT(options.max_num_final_iterations(), 0);
  options.set_global_sampling_ratio(
      parameter_dictionary->GetDouble("global_sampling_ratio"));
  options.set_log_residual_histograms(
      parameter_dictionary->GetBool("log_residual_histograms"));
  PopulateOverlappingSubmapsTrimmerOptions2D(&options, parameter_dictionary);
  return options;
}

LocalTrajectoryBuilderOptions2D CreateLocalTrajectoryBuilderOptions2D(
    common::ParameterDictionary* const parameter_dictionary) {
  LocalTrajectoryBuilderOptions2D options;
  options.set_min_range(parameter_dictionary->GetDouble("min_range"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  options.set_min_z(parameter_dictionary->GetDouble("min_z"));
  options.set_max_z(parameter_dictionary->GetDouble("max_z"));
  options.set_missing_data_ray_length(
      parameter_dictionary->GetDouble("missing_data_ray_length"));
  options.set_num_accumulated_range_data(
      parameter_dictionary->GetInt("num_accumulated_range_data"));
  options.set_voxel_filter_size(
      parameter_dictionary->GetDouble("voxel_filter_size"));
  *options.mutable_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary->GetDictionary("adaptive_voxel_filter").get());
  *options.mutable_loop_closure_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary
              ->GetDictionary("loop_closure_adaptive_voxel_filter")
              .get());
  if (parameter_dictionary->HasKey("use_online_correlative_scan_matching")) {
    options.set_use_online_correlative_scan_matching(
        parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  }
  if (parameter_dictionary->HasKey("real_time_correlative_scan_matcher")) {
    *options.mutable_real_time_correlative_scan_matcher_options() =
        scan_matching::CreateRealTimeCorrelativeScanMatcherOptions2D(
            parameter_dictionary
                ->GetDictionary("real_time_correlative_scan_matcher")
                .get());
  }
  *options.mutable_ceres_scan_matcher_options() =
      scan_matching::CreateCeresScanMatcherOptions2D(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_motion_filter_options() = CreateMotionFilterOptions(
      parameter_dictionary->GetDictionary("motion_filter").get());
  *options.mutable_pose_extrapolator_options() = CreatePoseExtrapolatorOptions(
      parameter_dictionary->GetDictionary("pose_extrapolator").get());
  *options.mutable_submaps_options() = CreateSubmapsOptions2D(
      parameter_dictionary->GetDictionary("submaps").get());
  return options;
}

namespace optimization {

OptimizationProblemOptions CreateOptimizationProblemOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  OptimizationProblemOptions options;
  options.set_huber_scale(parameter_dictionary->GetDouble("huber_scale"));
  options.set_odometry_translation_weight(
      parameter_dictionary->GetDouble("odometry_translation_weight"));
  options.set_odometry_rotation_weight(
      parameter_dictionary->GetDouble("odometry_rotation_weight"));
  options.set_local_slam_pose_translation_weight(
      parameter_dictionary->GetDouble("local_slam_pose_translation_weight"));
  options.set_local_slam_pose_rotation_weight(
      parameter_dictionary->GetDouble("local_slam_pose_rotation_weight"));
  options.set_log_solver_summary(
      parameter_dictionary->GetBool("log_solver_summary"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsConfig(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
