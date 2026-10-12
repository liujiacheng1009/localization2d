/*
 * Copyright 2016 The Cartographer Authors
 * Copyright 2026 The SweepNav Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include "cartographer/backend/ceres_pose_optimizer_2d.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "cartographer/application/slam_options.h"
#include "cartographer/foundation/utils.h"
#include "cartographer/scan_matching/cost_helpers.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

class SpaCostFunction2D {
 public:
  explicit SpaCostFunction2D(const Constraint::Pose& observed_relative_pose)
      : observed_relative_pose_(observed_relative_pose) {}

  template <typename T>
  bool operator()(const T* const start_pose, const T* const end_pose,
                  T* e) const {
    const std::array<T, 3> error = ScaleError(
        ComputeUnscaledError(observed_relative_pose_.zbar_ij, start_pose,
                             end_pose),
        observed_relative_pose_.translation_weight,
        observed_relative_pose_.rotation_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  const Constraint::Pose observed_relative_pose_;
};

ceres::CostFunction* CreateAutoDiffSpaCostFunction(
    const Constraint::Pose& observed_relative_pose) {
  return new ceres::AutoDiffCostFunction<SpaCostFunction2D, 3, 3, 3>(
      new SpaCostFunction2D(observed_relative_pose));
}

std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           transform::Yaw(pose)}};
}

transform::Rigid2d ToPose(const std::array<double, 3>& values) {
  return transform::MakeRigid2(Eigen::Vector2d(values[0], values[1]),
                               values[2]);
}

double ProcessCpuSeconds() {
  timespec cpu = {};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu);
  return cpu.tv_sec + 1e-9 * cpu.tv_nsec;
}

}  // namespace

CeresPoseOptimizer2D::CeresPoseOptimizer2D(
    const OptimizationProblemOptions& options)
    : PoseOptimizer2D(options, "ceres") {}

void CeresPoseOptimizer2D::SetMaxNumIterations(
    const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

void CeresPoseOptimizer2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, TrajectoryState>& trajectories_state) {
  if (node_data_.empty()) {
    return;
  }

  std::set<int> frozen_trajectories;
  for (const auto& it : trajectories_state) {
    if (it.second == TrajectoryState::FROZEN) {
      frozen_trajectories.insert(it.first);
    }
  }

  const auto wall_start = std::chrono::steady_clock::now();
  const double cpu_start = ProcessCpuSeconds();

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  MapById<SubmapId, std::array<double, 3>> C_submaps;
  MapById<NodeId, std::array<double, 3>> C_nodes;
  bool first_submap = true;
  for (const auto& submap_id_data : submap_data_) {
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    C_submaps.Insert(submap_id_data.id,
                     FromPose(submap_id_data.data.global_pose));
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);
    if (first_submap || frozen) {
      first_submap = false;
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
  }
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.global_pose_2d));
    problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
    }
  }
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        CreateAutoDiffSpaCostFunction(constraint.pose),
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps.at(constraint.submap_id).data(),
        C_nodes.at(constraint.node_id).data());
  }
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (frozen_trajectories.count(trajectory_id) != 0) {
      node_it = trajectory_end;
      continue;
    }

    auto prev_node_it = node_it;
    for (++node_it; node_it != trajectory_end; ++node_it) {
      const NodeId first_node_id = prev_node_it->id;
      const NodeSpec2D& first_node_data = prev_node_it->data;
      prev_node_it = node_it;
      const NodeId second_node_id = node_it->id;
      const NodeSpec2D& second_node_data = node_it->data;

      if (second_node_id.node_index != first_node_id.node_index + 1) {
        continue;
      }

      std::unique_ptr<transform::Rigid2d> relative_odometry =
          CalculateOdometryBetweenNodes(trajectory_id, first_node_data,
                                        second_node_data);
      if (relative_odometry != nullptr) {
        problem.AddResidualBlock(
            CreateAutoDiffSpaCostFunction(Constraint::Pose{
                *relative_odometry, options_.odometry_translation_weight(),
                options_.odometry_rotation_weight()}),
            nullptr, C_nodes.at(first_node_id).data(),
            C_nodes.at(second_node_id).data());
      }

      const transform::Rigid2d relative_local_slam_pose =
          first_node_data.local_pose_2d.inverse() *
          second_node_data.local_pose_2d;
      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(Constraint::Pose{
              relative_local_slam_pose,
              options_.local_slam_pose_translation_weight(),
              options_.local_slam_pose_rotation_weight()}),
          nullptr, C_nodes.at(first_node_id).data(),
          C_nodes.at(second_node_id).data());
    }
  }

  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);
  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
  }

  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        ToPose(C_submap_id_data.data);
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose_2d =
        ToPose(C_node_id_data.data);
  }

  const double wall_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    wall_start)
          .count();
  RecordSolveMetric(wall_seconds, ProcessCpuSeconds() - cpu_start,
                    static_cast<int>(constraints.size()));
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer
