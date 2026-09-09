/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/frontend/local_slam_2d.h"

#include <algorithm>
#include <limits>
#include <memory>

#include "absl/memory/memory.h"
#include "cartographer/foundation/sensor_data.h"

namespace cartographer {
namespace mapping {

LocalSlam2D::LocalSlam2D(
    const LocalTrajectoryBuilderOptions2D& options,
    std::string expected_range_sensor_id)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      expected_range_sensor_id_(std::move(expected_range_sensor_id)) {}

LocalSlam2D::~LocalSlam2D() {}

sensor::RangeData
LocalSlam2D::TransformToCurrentTrackingFrameAndFilter(
    const transform::Rigid3f& transform_to_current_tracking_frame,
    const sensor::RangeData& range_data) const {
  const sensor::RangeData cropped =
      sensor::CropRangeData(sensor::TransformRangeData(
                                range_data, transform_to_current_tracking_frame),
                            options_.min_z(), options_.max_z());
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()),
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
}

std::unique_ptr<transform::Rigid2d> LocalSlam2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_point_cloud) {
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  if (options_.use_online_correlative_scan_matching()) {
    real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_point_cloud, *matching_submap->grid(),
        &initial_ceres_pose);
  }
  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(pose_prediction.translation(), initial_ceres_pose,
                            filtered_point_cloud,
                            *matching_submap->grid(), pose_observation.get(),
                            &summary);
  return pose_observation;
}

std::unique_ptr<LocalSlam2D::MatchingResult>
LocalSlam2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& range_data) {
  CHECK_EQ(sensor_id, expected_range_sensor_id_);
  if (range_data.ranges.empty()) return nullptr;
  CHECK(std::is_sorted(
      range_data.ranges.begin(), range_data.ranges.end(),
      [](const sensor::TimedRangefinderPoint& lhs,
         const sensor::TimedRangefinderPoint& rhs) {
        return lhs.time < rhs.time;
      })) << "LaserScan points must be ordered by relative time.";

  const common::Time& time = range_data.time;
  InitializeExtrapolator(time);

  // TODO(gaschler): Check if this can strictly be 0.
  CHECK_LE(range_data.ranges.back().time, 0.f);
  const common::Time time_first_point =
      time + common::FromSeconds(range_data.ranges.front().time);
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  std::vector<transform::Rigid3f> range_data_poses;
  range_data_poses.reserve(range_data.ranges.size());
  bool warned = false;
  for (const auto& point : range_data.ranges) {
    common::Time time_point = time + common::FromSeconds(point.time);
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    range_data_poses.push_back(
        transform::Embed3D(
            extrapolator_->ExtrapolatePose(time_point).cast<float>()));
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is uninitialized until the last
    // accumulation.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  // Drop any returns below the minimum range and convert returns beyond the
  // maximum range into misses.
  for (size_t i = 0; i < range_data.ranges.size(); ++i) {
    const sensor::TimedRangefinderPoint& hit = range_data.ranges[i];
    const Eigen::Vector3f origin_in_local =
        range_data_poses[i] * range_data.origin;
    sensor::RangefinderPoint hit_in_local =
        range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    const Eigen::Vector3f delta = hit_in_local.position - origin_in_local;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        hit_in_local.position =
            origin_in_local +
            options_.missing_data_ray_length() / range * delta;
        accumulated_range_data_.misses.push_back(hit_in_local);
      }
    }
  }
  ++num_accumulated_;

  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    num_accumulated_ = 0;
    // TODO(gaschler): This assumes that 'range_data_poses.back()' is at time
    // 'time'.
    accumulated_range_data_.origin = range_data_poses.back().translation();
    return AddAccumulatedRangeData(
        time,
        TransformToCurrentTrackingFrameAndFilter(
            range_data_poses.back().inverse(), accumulated_range_data_));
  }
  return nullptr;
}

std::unique_ptr<LocalSlam2D::MatchingResult>
LocalSlam2D::AddAccumulatedRangeData(
    const common::Time time, const sensor::RangeData& tracking_range_data) {
  if (tracking_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  const transform::Rigid2d pose_prediction = extrapolator_->ExtrapolatePose(time);

  const sensor::PointCloud& filtered_point_cloud =
      sensor::AdaptiveVoxelFilter(tracking_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_point_cloud.empty()) {
    return nullptr;
  }

  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_point_cloud);
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  extrapolator_->AddPose(time, *pose_estimate_2d);

  sensor::RangeData range_data_in_local =
      TransformRangeData(tracking_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_point_cloud,
      *pose_estimate_2d);

  return absl::make_unique<MatchingResult>(
      MatchingResult{time, *pose_estimate_2d, std::move(range_data_in_local),
                     std::move(insertion_result)});
}

std::unique_ptr<LocalSlam2D::InsertionResult>
LocalSlam2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_point_cloud,
    const transform::Rigid2d& pose_estimate) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time, filtered_point_cloud, pose_estimate}),
      std::move(insertion_submaps)});
}

void LocalSlam2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

void LocalSlam2D::InitializeExtrapolator(const common::Time time) {
  if (extrapolator_ != nullptr) {
    return;
  }
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                              .constant_velocity()
                                              .pose_queue_duration()));
  extrapolator_->AddPose(time, transform::Rigid2d());
}

}  // namespace mapping
}  // namespace cartographer
