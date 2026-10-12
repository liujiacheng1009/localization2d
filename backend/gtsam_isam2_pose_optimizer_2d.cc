// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/backend/gtsam_isam2_pose_optimizer_2d.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <set>

#include "glog/logging.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using gtsam::symbol_shorthand::N;
using gtsam::symbol_shorthand::S;

gtsam::Pose2 ToGtsam(const transform::Rigid2d& pose) {
  return gtsam::Pose2(pose.translation().x(), pose.translation().y(),
                      transform::Yaw(pose));
}

transform::Rigid2d FromGtsam(const gtsam::Pose2& pose) {
  return transform::MakeRigid2(Eigen::Vector2d(pose.x(), pose.y()),
                               pose.theta());
}

gtsam::SharedNoiseModel NoiseFromWeights(const double translation_weight,
                                         const double rotation_weight) {
  const double tx = std::max(translation_weight, 1e-6);
  const double ty = tx;
  const double rz = std::max(rotation_weight, 1e-6);
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector3() << 1. / tx, 1. / ty, 1. / rz).finished());
}

gtsam::SharedNoiseModel ConstraintNoise(const Constraint::Pose& pose,
                                        const bool robust,
                                        const double huber_scale) {
  auto gaussian =
      NoiseFromWeights(pose.translation_weight, pose.rotation_weight);
  if (!robust) {
    return gaussian;
  }
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(huber_scale), gaussian);
}

double ProcessCpuSeconds() {
  timespec cpu = {};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu);
  return cpu.tv_sec + 1e-9 * cpu.tv_nsec;
}

gtsam::Key SubmapKey(const SubmapId& id) {
  return S(static_cast<std::uint64_t>(id.trajectory_id) << 32 |
           static_cast<std::uint32_t>(id.submap_index));
}

gtsam::Key NodeKey(const NodeId& id) {
  return N(static_cast<std::uint64_t>(id.trajectory_id) << 32 |
           static_cast<std::uint32_t>(id.node_index));
}

gtsam::SharedNoiseModel PriorNoise() {
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector3() << 1e-6, 1e-6, 1e-6).finished());
}

}  // namespace

GtsamIsam2PoseOptimizer2D::GtsamIsam2PoseOptimizer2D(
    const OptimizationProblemOptions& options)
    : PoseOptimizer2D(options, "gtsam_isam2") {
  isam2_params_.relinearizeThreshold =
      options.gtsam_options().relinearize_threshold() > 0.
          ? options.gtsam_options().relinearize_threshold()
          : 0.1;
  isam2_params_.relinearizeSkip =
      options.gtsam_options().relinearize_skip() > 0
          ? options.gtsam_options().relinearize_skip()
          : 10;
  isam2_params_.enablePartialRelinearizationCheck = true;
  isam2_ = gtsam::ISAM2(isam2_params_);
}

void GtsamIsam2PoseOptimizer2D::ResetIsam2() {
  isam2_ = gtsam::ISAM2(isam2_params_);
  inserted_submaps_.clear();
  inserted_nodes_.clear();
  constraints_fed_ = 0;
  sequential_edges_.clear();
  first_submap_anchored_ = false;
}

void GtsamIsam2PoseOptimizer2D::TrimTrajectoryNode(const NodeId& node_id) {
  PoseOptimizer2D::TrimTrajectoryNode(node_id);
  ResetIsam2();
}

void GtsamIsam2PoseOptimizer2D::TrimSubmap(const SubmapId& submap_id) {
  PoseOptimizer2D::TrimSubmap(submap_id);
  ResetIsam2();
}

void GtsamIsam2PoseOptimizer2D::SetMaxNumIterations(
    const int32 max_num_iterations) {
  // Final optimization: a few extra empty updates to tighten the Bayes tree.
  extra_update_passes_ = std::max(0, max_num_iterations / 10);
}

void GtsamIsam2PoseOptimizer2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, TrajectoryState>& trajectories_state) {
  if (node_data_.empty() || submap_data_.empty()) {
    return;
  }

  // Constraint list shrank (trim) or ids removed → full ISAM2 rebuild.
  bool need_reset = constraints_fed_ > static_cast<int>(constraints.size());
  if (!need_reset) {
    for (const SubmapId& id : inserted_submaps_) {
      if (!submap_data_.Contains(id)) {
        need_reset = true;
        break;
      }
    }
  }
  if (!need_reset) {
    for (const NodeId& id : inserted_nodes_) {
      if (!node_data_.Contains(id)) {
        need_reset = true;
        break;
      }
    }
  }
  if (need_reset) {
    ResetIsam2();
  }

  std::set<int> frozen_trajectories;
  for (const auto& it : trajectories_state) {
    if (it.second == TrajectoryState::FROZEN) {
      frozen_trajectories.insert(it.first);
    }
  }

  const auto wall_start = std::chrono::steady_clock::now();
  const double cpu_start = ProcessCpuSeconds();

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  for (const auto& submap_id_data : submap_data_) {
    if (inserted_submaps_.count(submap_id_data.id) != 0) {
      continue;
    }
    const gtsam::Key key = SubmapKey(submap_id_data.id);
    const gtsam::Pose2 pose = ToGtsam(submap_id_data.data.global_pose);
    new_values.insert(key, pose);
    inserted_submaps_.insert(submap_id_data.id);
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    if (!first_submap_anchored_ || frozen) {
      first_submap_anchored_ = true;
      new_factors.add(
          gtsam::PriorFactor<gtsam::Pose2>(key, pose, PriorNoise()));
    }
  }

  for (const auto& node_id_data : node_data_) {
    if (inserted_nodes_.count(node_id_data.id) != 0) {
      continue;
    }
    const gtsam::Key key = NodeKey(node_id_data.id);
    const gtsam::Pose2 pose = ToGtsam(node_id_data.data.global_pose_2d);
    new_values.insert(key, pose);
    inserted_nodes_.insert(node_id_data.id);
    if (frozen_trajectories.count(node_id_data.id.trajectory_id) != 0) {
      new_factors.add(
          gtsam::PriorFactor<gtsam::Pose2>(key, pose, PriorNoise()));
    }
  }

  for (int i = constraints_fed_; i < static_cast<int>(constraints.size());
       ++i) {
    const Constraint& constraint = constraints[i];
    new_factors.add(gtsam::BetweenFactor<gtsam::Pose2>(
        SubmapKey(constraint.submap_id), NodeKey(constraint.node_id),
        ToGtsam(constraint.pose.zbar_ij),
        ConstraintNoise(constraint.pose,
                        constraint.tag == Constraint::INTER_SUBMAP,
                        options_.huber_scale())));
  }
  constraints_fed_ = static_cast<int>(constraints.size());

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
      const auto edge_key = std::make_pair(first_node_id, second_node_id);
      if (sequential_edges_.count(edge_key) != 0) {
        continue;
      }
      sequential_edges_.insert(edge_key);

      std::unique_ptr<transform::Rigid2d> relative_odometry =
          CalculateOdometryBetweenNodes(trajectory_id, first_node_data,
                                        second_node_data);
      if (relative_odometry != nullptr) {
        new_factors.add(gtsam::BetweenFactor<gtsam::Pose2>(
            NodeKey(first_node_id), NodeKey(second_node_id),
            ToGtsam(*relative_odometry),
            NoiseFromWeights(options_.odometry_translation_weight(),
                             options_.odometry_rotation_weight())));
      }

      const transform::Rigid2d relative_local_slam_pose =
          first_node_data.local_pose_2d.inverse() *
          second_node_data.local_pose_2d;
      new_factors.add(gtsam::BetweenFactor<gtsam::Pose2>(
          NodeKey(first_node_id), NodeKey(second_node_id),
          ToGtsam(relative_local_slam_pose),
          NoiseFromWeights(options_.local_slam_pose_translation_weight(),
                           options_.local_slam_pose_rotation_weight())));
    }
  }

  if (!new_factors.empty() || !new_values.empty()) {
    isam2_.update(new_factors, new_values);
  } else {
    // No new factors (e.g. final opt only) — still allow extra passes.
    isam2_.update();
  }
  for (int i = 0; i < extra_update_passes_; ++i) {
    isam2_.update();
  }
  extra_update_passes_ = 0;

  const gtsam::Values result = isam2_.calculateEstimate();
  if (options_.log_solver_summary()) {
    LOG(INFO) << "GTSAM ISAM2 variables=" << result.size()
              << " constraints_fed=" << constraints_fed_;
  }

  for (const auto& submap_id_data : submap_data_) {
    submap_data_.at(submap_id_data.id).global_pose =
        FromGtsam(result.at<gtsam::Pose2>(SubmapKey(submap_id_data.id)));
  }
  for (const auto& node_id_data : node_data_) {
    node_data_.at(node_id_data.id).global_pose_2d =
        FromGtsam(result.at<gtsam::Pose2>(NodeKey(node_id_data.id)));
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
