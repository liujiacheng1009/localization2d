/*
 * Copyright 2016 The Cartographer Authors
 * Copyright 2026 The SweepNav Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include "cartographer/backend/pose_optimizer_2d.h"

#include <time.h>

#include "absl/memory/memory.h"
#include "cartographer/backend/ceres_pose_optimizer_2d.h"
#include "cartographer/backend/gtsam_isam2_pose_optimizer_2d.h"
#include "cartographer/backend/gtsam_pose_optimizer_2d.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/foundation/utils.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace optimization {

PoseOptimizer2D::PoseOptimizer2D(const OptimizationProblemOptions& options,
                                 std::string backend_name)
    : options_(options), backend_name_(std::move(backend_name)) {}

PoseOptimizer2D::~PoseOptimizer2D() = default;

void PoseOptimizer2D::AddOdometryData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  odometry_data_.Append(trajectory_id, odometry_data);
}

void PoseOptimizer2D::AddTrajectoryNode(const int trajectory_id,
                                        const NodeSpec2D& node_data) {
  node_data_.Append(trajectory_id, node_data);
}

void PoseOptimizer2D::InsertTrajectoryNode(const NodeId& node_id,
                                           const NodeSpec2D& node_data) {
  node_data_.Insert(node_id, node_data);
}

void PoseOptimizer2D::TrimTrajectoryNode(const NodeId& node_id) {
  odometry_data_.Trim(node_data_, node_id);
  node_data_.Trim(node_id);
}

void PoseOptimizer2D::AddSubmap(const int trajectory_id,
                                const transform::Rigid2d& global_submap_pose) {
  submap_data_.Append(trajectory_id, SubmapSpec2D{global_submap_pose});
}

void PoseOptimizer2D::InsertSubmap(
    const SubmapId& submap_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Insert(submap_id, SubmapSpec2D{global_submap_pose});
}

void PoseOptimizer2D::TrimSubmap(const SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}

void PoseOptimizer2D::RecordSolveMetric(const double wall_seconds,
                                        const double cpu_seconds,
                                        const int num_constraints) {
  SolveMetric metric;
  metric.index = static_cast<int>(solve_metrics_.size());
  metric.wall_seconds = wall_seconds;
  metric.cpu_seconds = cpu_seconds;
  metric.num_nodes = node_data_.size();
  metric.num_submaps = submap_data_.size();
  metric.num_constraints = num_constraints;
  solve_metrics_.push_back(metric);
}

std::unique_ptr<transform::Rigid2d> PoseOptimizer2D::InterpolateOdometry(
    const int trajectory_id, const common::Time time) const {
  const auto it = odometry_data_.lower_bound(trajectory_id, time);
  if (it == odometry_data_.EndOfTrajectory(trajectory_id)) {
    return nullptr;
  }
  if (it == odometry_data_.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid2d>(it->pose);
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  const double factor = common::ToSeconds(time - prev_it->time) /
                        common::ToSeconds(it->time - prev_it->time);
  const Eigen::Vector2d translation =
      prev_it->pose.translation() +
      factor * (it->pose.translation() - prev_it->pose.translation());
  const double yaw =
      transform::Yaw(prev_it->pose) +
      factor * common::NormalizeAngleDifference(
                   transform::Yaw(it->pose) - transform::Yaw(prev_it->pose));
  return absl::make_unique<transform::Rigid2d>(yaw, translation);
}

std::unique_ptr<transform::Rigid2d>
PoseOptimizer2D::CalculateOdometryBetweenNodes(
    const int trajectory_id, const NodeSpec2D& first_node_data,
    const NodeSpec2D& second_node_data) const {
  if (odometry_data_.HasTrajectory(trajectory_id)) {
    const std::unique_ptr<transform::Rigid2d> first_node_odometry =
        InterpolateOdometry(trajectory_id, first_node_data.time);
    const std::unique_ptr<transform::Rigid2d> second_node_odometry =
        InterpolateOdometry(trajectory_id, second_node_data.time);
    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      const transform::Rigid2d relative_odometry =
          first_node_odometry->inverse() * (*second_node_odometry);
      return absl::make_unique<transform::Rigid2d>(relative_odometry);
    }
  }
  return nullptr;
}

std::unique_ptr<PoseOptimizer2D> CreatePoseOptimizer2D(
    const PoseGraphOptions& pose_graph_options) {
  const std::string& name = pose_graph_options.optimizer();
  const auto& opt = pose_graph_options.optimization_problem_options();
  if (name == "ceres") {
    return absl::make_unique<CeresPoseOptimizer2D>(opt);
  }
  if (name == "gtsam") {
    return absl::make_unique<GtsamBatchPoseOptimizer2D>(opt);
  }
  if (name == "gtsam_isam2" || name.empty()) {
    return absl::make_unique<GtsamIsam2PoseOptimizer2D>(opt);
  }
  LOG(FATAL) << "Unknown pose_graph.optimizer '" << name
             << "' (expected ceres|gtsam|gtsam_isam2)";
  return nullptr;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
