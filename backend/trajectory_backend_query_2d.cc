// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/backend/trajectory_backend_2d.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <vector>

#include "cartographer/foundation/utils.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/mapping/submap_2d.h"
#include "cartographer/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr float kRelocalizationResolution = 0.05f;
constexpr float kOccupiedProbability = 0.9f;
constexpr float kFreeProbability = 0.1f;
constexpr float kSearchFloor = 0.45f;
constexpr float kMinAcceptScore = 0.68f;
constexpr float kMinScoreMargin = 0.10f;
constexpr double kMinJumpMeters = 0.8;
constexpr double kDistinctHypothesisMeters = 1.0;
constexpr float kSuppressRadius = 0.55f;
constexpr float kMinPointRange = 0.15f;
constexpr float kMaxPointRange = 11.8f;

struct RelocalizationSample {
  Eigen::Vector2f center;
  bool occupied = false;
};

void PaintIfUnknown(ProbabilityGrid* grid, const MapLimits& limits,
                    const Eigen::Array2i& cell, float probability) {
  if (!limits.Contains(cell) || grid->IsKnown(cell)) return;
  grid->SetProbability(cell, probability);
}

std::unique_ptr<ProbabilityGrid> BuildRelocalizationGrid(
    const std::vector<RelocalizationSample>& samples,
    const std::vector<Eigen::Vector2f>* suppressed_hits,
    ValueConversionTables* conversion_tables) {
  bool any = false;
  Eigen::Vector2f min_p = Eigen::Vector2f::Zero();
  Eigen::Vector2f max_p = Eigen::Vector2f::Zero();
  const float suppress_sq = kSuppressRadius * kSuppressRadius;
  auto kept = [&](const Eigen::Vector2f& center) {
    if (suppressed_hits == nullptr) return true;
    for (const Eigen::Vector2f& hit : *suppressed_hits) {
      if ((center - hit).squaredNorm() <= suppress_sq) return false;
    }
    return true;
  };
  for (const RelocalizationSample& sample : samples) {
    if (!kept(sample.center)) continue;
    if (!any) {
      min_p = max_p = sample.center;
      any = true;
    } else {
      min_p = min_p.cwiseMin(sample.center);
      max_p = max_p.cwiseMax(sample.center);
    }
  }
  if (!any) return nullptr;
  min_p.array() -= 1.f;
  max_p.array() += 1.f;
  if ((max_p - min_p).maxCoeff() > 80.f) return nullptr;
  const int cells_along_x = std::max(
      1, static_cast<int>(std::ceil((max_p.x() - min_p.x()) /
                                   kRelocalizationResolution)));
  const int cells_along_y = std::max(
      1, static_cast<int>(std::ceil((max_p.y() - min_p.y()) /
                                   kRelocalizationResolution)));
  const MapLimits limits(
      kRelocalizationResolution,
      Eigen::Vector2d(min_p.x() + cells_along_x * kRelocalizationResolution,
                      min_p.y() + cells_along_y * kRelocalizationResolution),
      CellLimits(cells_along_y, cells_along_x));
  auto grid = std::make_unique<ProbabilityGrid>(limits, conversion_tables);
  for (const RelocalizationSample& sample : samples) {
    if (!sample.occupied || !kept(sample.center)) continue;
    const Eigen::Array2i cell = limits.GetCellIndex(sample.center);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        PaintIfUnknown(grid.get(), limits,
                       Eigen::Array2i(cell.x() + dx, cell.y() + dy),
                       kOccupiedProbability);
      }
    }
  }
  for (const RelocalizationSample& sample : samples) {
    if (sample.occupied || !kept(sample.center)) continue;
    PaintIfUnknown(grid.get(), limits, limits.GetCellIndex(sample.center),
                   kFreeProbability);
  }
  if (grid->known_cells_box_for_serialization().isEmpty()) return nullptr;
  return grid;
}

struct RelocalizationHit {
  bool found = false;
  float score = 0.f;
  transform::Rigid2d pose = transform::Rigid2d();
};

RelocalizationHit MatchMergedGrid(
    const ProbabilityGrid& grid, const sensor::PointCloud& cloud,
    const FastCorrelativeScanMatcherOptions2D& options) {
  scan_matching::FastCorrelativeScanMatcher2D matcher(grid, options);
  RelocalizationHit hit;
  hit.found = matcher.MatchFullSubmap(cloud, kSearchFloor, &hit.score, &hit.pose);
  return hit;
}

}  // namespace

TrajectoryBackend2D::Relocalization TrajectoryBackend2D::Relocalize(
    const sensor::PointCloud& cloud,
    const transform::Rigid2d& current_local) const {
  const sensor::PointCloud usable = cloud.copy_if([](const sensor::RangefinderPoint& point) {
    const float range = point.position.head<2>().norm();
    return range >= kMinPointRange && range <= kMaxPointRange;
  });
  if (usable.size() < 30) return Relocalization{};
  std::vector<RelocalizationSample> samples;
  {
    absl::MutexLock locker(&mutex_);
    for (const auto& submap_id_data : data_.submap_data) {
      auto submap = std::dynamic_pointer_cast<const Submap2D>(
          submap_id_data.data.submap);
      if (!submap || submap->grid() == nullptr) continue;
      if (submap->num_range_data() < 8) continue;
      const auto* probability_grid =
          dynamic_cast<const ProbabilityGrid*>(submap->grid());
      if (probability_grid == nullptr) continue;
      const Grid2D& grid = *probability_grid;
      const Eigen::AlignedBox2i& box = grid.known_cells_box_for_serialization();
      if (box.isEmpty()) continue;
      for (int ix = box.min().x(); ix <= box.max().x(); ++ix) {
        for (int iy = box.min().y(); iy <= box.max().y(); ++iy) {
          const Eigen::Array2i cell(ix, iy);
          if (!grid.IsKnown(cell)) continue;
          const float probability = probability_grid->GetProbability(cell);
          if (probability >= 0.52f) {
            samples.push_back(
                RelocalizationSample{grid.limits().GetCellCenter(cell), true});
          } else if (probability <= 0.48f) {
            samples.push_back(
                RelocalizationSample{grid.limits().GetCellCenter(cell), false});
          }
        }
      }
    }
  }
  if (samples.empty()) return Relocalization{};
  ValueConversionTables conversion_tables;
  const auto merged =
      BuildRelocalizationGrid(samples, nullptr, &conversion_tables);
  if (merged == nullptr) return Relocalization{};
  const auto& matcher_options = options_.constraint_builder_options()
                                    .fast_correlative_scan_matcher_options();
  const RelocalizationHit best = MatchMergedGrid(*merged, usable, matcher_options);
  if (!best.found || best.score < kMinAcceptScore) {
    LOG(INFO) << "Relocalization rejected, best score "
              << (best.found ? best.score : 0.f);
    return Relocalization{};
  }
  std::vector<Eigen::Vector2f> hits;
  hits.reserve(usable.size());
  for (const sensor::RangefinderPoint& point : usable) {
    hits.push_back(
        (best.pose * point.position.head<2>().cast<double>()).cast<float>());
  }
  const auto suppressed =
      BuildRelocalizationGrid(samples, &hits, &conversion_tables);
  float second_score = 0.f;
  bool distinct = false;
  if (suppressed != nullptr) {
    const RelocalizationHit second =
        MatchMergedGrid(*suppressed, usable, matcher_options);
    if (second.found) {
      second_score = second.score;
      distinct = (second.pose.translation() - best.pose.translation()).norm() >=
                 kDistinctHypothesisMeters;
    }
  }
  const float margin = distinct ? best.score - second_score : best.score;
  if (distinct && margin < kMinScoreMargin) {
    LOG(INFO) << "Relocalization rejected, score " << best.score
              << " second " << second_score << " at ("
              << best.pose.translation().x() << ", "
              << best.pose.translation().y() << ")";
    return Relocalization{};
  }
  LOG(INFO) << "Relocalization score " << best.score << " second " << second_score
            << " at (" << best.pose.translation().x() << ", "
            << best.pose.translation().y() << ") yaw "
            << transform::Yaw(best.pose);
  if ((best.pose.translation() - current_local.translation()).norm() <
      kMinJumpMeters) {
    LOG(INFO) << "Relocalization stays, match is already near the held pose";
    return {Relocalization::Decision::kStay, best.pose};
  }
  return {Relocalization::Decision::kJump, best.pose};
}

MapById<NodeId, TrajectoryNode> TrajectoryBackend2D::GetTrajectoryNodes() const {
  absl::MutexLock locker(&mutex_);
  return data_.trajectory_nodes;
}

MapById<NodeId, TrajectoryNodePose> TrajectoryBackend2D::GetTrajectoryNodePoses() const {
  MapById<NodeId, TrajectoryNodePose> node_poses;
  absl::MutexLock locker(&mutex_);
  for (const auto& node_id_data : data_.trajectory_nodes) {
    absl::optional<TrajectoryNodePose::ConstantPoseData> constant_pose_data;
    if (node_id_data.data.constant_data != nullptr) {
      constant_pose_data = TrajectoryNodePose::ConstantPoseData{
          node_id_data.data.constant_data->time,
          node_id_data.data.constant_data->local_pose};
    }
    node_poses.Insert(
        node_id_data.id,
        TrajectoryNodePose{node_id_data.data.global_pose, constant_pose_data});
  }
  return node_poses;
}

std::map<int, TrajectoryState>
TrajectoryBackend2D::GetTrajectoryStates() const {
  std::map<int, TrajectoryState> trajectories_state;
  absl::MutexLock locker(&mutex_);
  for (const auto& it : data_.trajectories_state) {
    trajectories_state[it.first] = it.second.state;
  }
  return trajectories_state;
}

sensor::MapByTime<sensor::OdometryData> TrajectoryBackend2D::GetOdometryData() const {
  absl::MutexLock locker(&mutex_);
  return optimization_problem_->odometry_data();
}

std::vector<Constraint> TrajectoryBackend2D::constraints() const {
  std::vector<Constraint> result;
  absl::MutexLock locker(&mutex_);
  result = data_.constraints;
  return result;
}

void TrajectoryBackend2D::SetInitialTrajectoryPose(const int from_trajectory_id,
                                         const int to_trajectory_id,
                                         const transform::Rigid2d& pose,
                                         const common::Time time) {
  absl::MutexLock locker(&mutex_);
  data_.initial_trajectory_poses[from_trajectory_id] =
      InitialTrajectoryPose{to_trajectory_id, pose, time};
}

transform::Rigid2d TrajectoryBackend2D::GetInterpolatedGlobalTrajectoryPose(
    const int trajectory_id, const common::Time time) const {
  CHECK_GT(data_.trajectory_nodes.SizeOfTrajectoryOrZero(trajectory_id), 0);
  const auto it = data_.trajectory_nodes.lower_bound(trajectory_id, time);
  if (it == data_.trajectory_nodes.BeginOfTrajectory(trajectory_id)) {
    return it->data.global_pose;
  }
  if (it == data_.trajectory_nodes.EndOfTrajectory(trajectory_id)) {
    return std::prev(it)->data.global_pose;
  }
  const auto& start = *std::prev(it);
  const auto& end = *it;
  const double factor = common::ToSeconds(time - start.data.time()) /
                        common::ToSeconds(end.data.time() - start.data.time());
  const Eigen::Vector2d translation =
      start.data.global_pose.translation() +
      factor * (end.data.global_pose.translation() -
                start.data.global_pose.translation());
  const double delta_angle = common::NormalizeAngleDifference(
      transform::Yaw(end.data.global_pose) -
      transform::Yaw(start.data.global_pose));
  return transform::MakeRigid2(
      translation, transform::Yaw(start.data.global_pose) + factor * delta_angle);
}

transform::Rigid2d TrajectoryBackend2D::GetLocalToGlobalTransform(
    const int trajectory_id) const {
  absl::MutexLock locker(&mutex_);
  return ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                       trajectory_id);
}

SubmapData TrajectoryBackend2D::GetSubmapData(
    const SubmapId& submap_id) const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock(submap_id);
}

MapById<SubmapId, SubmapData>
TrajectoryBackend2D::GetAllSubmapData() const {
  absl::MutexLock locker(&mutex_);
  return GetSubmapDataUnderLock();
}

MapById<SubmapId, SubmapPose>
TrajectoryBackend2D::GetAllSubmapPoses() const {
  absl::MutexLock locker(&mutex_);
  MapById<SubmapId, SubmapPose> submap_poses;
  for (const auto& submap_id_data : data_.submap_data) {
    auto submap_data = GetSubmapDataUnderLock(submap_id_data.id);
    submap_poses.Insert(
        submap_id_data.id,
        SubmapPose{submap_data.submap->num_range_data(), submap_data.pose});
  }
  return submap_poses;
}

transform::Rigid2d TrajectoryBackend2D::ComputeLocalToGlobalTransform(
    const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
    const int trajectory_id) const {
  auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
  if (begin_it == end_it) {
    const auto it = data_.initial_trajectory_poses.find(trajectory_id);
    if (it == data_.initial_trajectory_poses.end()) {
      return transform::Rigid2d();
    }
    return GetInterpolatedGlobalTrajectoryPose(it->second.to_trajectory_id,
                                               it->second.time) *
           it->second.relative_pose;
  }
  const SubmapId last_optimized_submap_id = std::prev(end_it)->id;
  return global_submap_poses.at(last_optimized_submap_id).global_pose *
         data_.submap_data.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

SubmapData TrajectoryBackend2D::GetSubmapDataUnderLock(
    const SubmapId& submap_id) const {
  const auto it = data_.submap_data.find(submap_id);
  if (it == data_.submap_data.end()) return {};
  auto submap = it->data.submap;
  if (data_.global_submap_poses_2d.Contains(submap_id)) {
    return {submap,
            data_.global_submap_poses_2d.at(submap_id).global_pose};
  }
  return {submap, ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                                submap_id.trajectory_id) *
                      submap->local_pose()};
}

}  // namespace mapping
}  // namespace cartographer
