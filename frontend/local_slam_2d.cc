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
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>

#include "absl/memory/memory.h"
#include "cartographer/foundation/sensor_data.h"

namespace cartographer {
namespace mapping {
namespace {

// On the 160418 run the chassis sat near 1 degree while driving. Sustained
// tilts of 15 to 23 degrees were the lifts and the threshold crossings.
// 12 degrees starts the hold; 6 degrees is level enough to search again.
constexpr double kTiltEnterRadians = 12. * std::numbers::pi / 180.;
constexpr double kTiltExitRadians = 6. * std::numbers::pi / 180.;

}  // namespace

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

  // Destort the whole sweep into the last-beam tracking frame, then lift that
  // rigid motion into the local frame with the predicted end pose. Motion comes
  // from odometry over this sweep (constant-twist fallback), not from clamping
  // early beams to the last SLAM pose.
  const transform::Rigid2d pose_at_end = extrapolator_->ExtrapolatePose(time);
  if (std::abs(range_data.ranges.front().time) > 1e-4) {
    const transform::Rigid2d first_in_end = extrapolator_->TrackingMotion(
        time + common::FromSeconds(range_data.ranges.front().time), time);
    const double destort_yaw = transform::Yaw(first_in_end);
    if (std::abs(destort_yaw) > 0.08) {
      LOG(INFO) << "Scan destortion yaw=" << destort_yaw << " rad over "
                << -range_data.ranges.front().time << " s";
    }
  }

  std::vector<transform::Rigid3f> range_data_poses;
  range_data_poses.reserve(range_data.ranges.size());
  for (const auto& point : range_data.ranges) {
    const common::Time time_point = time + common::FromSeconds(point.time);
    const transform::Rigid2d point_in_end =
        extrapolator_->TrackingMotion(time_point, time);
    range_data_poses.push_back(transform::Embed3D(
        (pose_at_end * point_in_end).cast<float>()));
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

  const common::Time last_pose_time = extrapolator_->GetLastPoseTime();
  const double gap_seconds = common::ToSeconds(time - last_pose_time);
  const double odom_interval_yaw = std::abs(
      transform::Yaw(extrapolator_->TrackingMotion(time, last_pose_time)));
  const bool gap_skip = has_range_pose_ &&
                        options_.skip_insert_after_gap_seconds() > 0. &&
                        gap_seconds > options_.skip_insert_after_gap_seconds();
  const transform::Rigid2d pose_prediction = extrapolator_->ExtrapolatePose(time);

  const sensor::PointCloud& filtered_point_cloud =
      sensor::AdaptiveVoxelFilter(tracking_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_point_cloud.empty()) {
    return nullptr;
  }

  const double tilt = chassis_tilt_.load();
  if (tilt > kTiltEnterRadians) {
    if (tilt_high_since_ == common::Time::min()) tilt_high_since_ = time;
  } else {
    tilt_high_since_ = common::Time::min();
  }
  // A single IMU attitude spike while turning is not evidence that the
  // chassis was lifted. Require a full second above the enter threshold
  // before latching the hold; otherwise a transient tilt can freeze mapping
  // and make every subsequent relocalization attempt fail.
  const bool sustained_tilt =
      tilt_high_since_ != common::Time::min() &&
      common::ToSeconds(time - tilt_high_since_) >= 1.0;
  if (has_range_pose_ && sustained_tilt && !suspended_) {
    suspended_ = true;
    LOG(INFO) << "Chassis tilted " << tilt
              << " rad; holding the last pose and not inserting";
  }
  // A tilted or lifted chassis must not add a keyframe. Relocalization waits
  // until the chassis is level again; a tilted scan would lock onto a wall.
  if (has_range_pose_ && suspended_) {
    return HoldLastPose(time, tracking_range_data, filtered_point_cloud,
                        tilt < kTiltExitRadians);
  }

  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_point_cloud);
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  // The 0.15 m local window walks toward whichever wall fragment scores
  // least badly. That step must not be inserted. Wheels still in translation
  // and yaw means the chassis was lifted while level: latch the hold and
  // relocalize. An in-place spin, or a match that outruns a real wheel step,
  // follows the wheel prediction and does not start a search.
  if (has_range_pose_) {
    const transform::Rigid2d from_last =
        extrapolator_->GetLastPose().inverse() * pose_prediction;
    const transform::Rigid2d from_prediction =
        pose_prediction.inverse() * *pose_estimate_2d;
    const bool translation_still = from_last.translation().norm() < 0.05;
    const bool yaw_still = std::abs(transform::Yaw(from_last)) < 0.08;
    const bool walked = from_prediction.translation().norm() > 0.12;
    if (walked && translation_still && yaw_still) {
      if (!suspended_) {
        LOG(INFO) << "Chassis lifted with the wheels still; holding the last "
                     "pose until it is level and relocalized";
      }
      suspended_ = true;
      return HoldLastPose(time, tracking_range_data, filtered_point_cloud,
                          true);
    }
    if (walked) {
      const transform::Rigid2d held = pose_prediction;
      extrapolator_->AddPose(time, held);
      has_range_pose_ = true;
      sensor::RangeData held_range = TransformRangeData(
          tracking_range_data, transform::Embed3D(held.cast<float>()));
      MatchingResult result{time, held, std::move(held_range), nullptr};
      return absl::make_unique<MatchingResult>(std::move(result));
    }
  }
  const double matched_interval_yaw = std::abs(transform::Yaw(
      extrapolator_->GetLastPose().inverse() * *pose_estimate_2d));
  const double turning_limit = options_.skip_insert_while_turning_radians();
  const bool turning_skip =
      turning_limit > 0. &&
      (odom_interval_yaw > turning_limit ||
       matched_interval_yaw > turning_limit);
  const bool skip_insert = gap_skip || turning_skip;
  extrapolator_->AddPose(time, *pose_estimate_2d);
  has_range_pose_ = true;

  sensor::RangeData range_data_in_local =
      TransformRangeData(tracking_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  std::unique_ptr<InsertionResult> insertion_result;
  if (skip_insert) {
    LOG(INFO) << "Skipping submap insert gap=" << gap_seconds
              << "s odom_yaw=" << odom_interval_yaw
              << " matched_yaw=" << matched_interval_yaw;
  } else {
    insertion_result = InsertIntoSubmap(
        time, range_data_in_local, filtered_point_cloud,
        *pose_estimate_2d);
  }

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

std::unique_ptr<LocalSlam2D::InsertionResult>
LocalSlam2D::CommitRelocalizedPose(
    const common::Time time, const transform::Rigid2d& local_pose,
    const sensor::RangeData& tracking_range_data) {
  CHECK(extrapolator_ != nullptr);
  suspended_ = false;
  extrapolator_->AddPose(time, local_pose);
  has_range_pose_ = true;
  active_submaps_.ReplaceWithNewOrigin(local_pose.translation().cast<float>());
  const sensor::PointCloud filtered = sensor::AdaptiveVoxelFilter(
      tracking_range_data.returns, options_.adaptive_voxel_filter_options());
  const sensor::RangeData range_data_in_local = TransformRangeData(
      tracking_range_data, transform::Embed3D(local_pose.cast<float>()));
  LOG(INFO) << "Relocalized local pose to (" << local_pose.translation().x()
            << ", " << local_pose.translation().y() << ")";
  return InsertIntoSubmap(time, range_data_in_local, filtered, local_pose);
}

void LocalSlam2D::SetChassisTilt(const double tilt_radians) {
  chassis_tilt_.store(tilt_radians);
}

std::unique_ptr<LocalSlam2D::MatchingResult> LocalSlam2D::HoldLastPose(
    const common::Time time, const sensor::RangeData& tracking_range_data,
    const sensor::PointCloud& filtered_point_cloud, const bool relocalize) {
  const transform::Rigid2d held = extrapolator_->GetLastPose();
  sensor::RangeData held_range = TransformRangeData(
      tracking_range_data, transform::Embed3D(held.cast<float>()));
  MatchingResult result{time, held, std::move(held_range), nullptr};
  if (relocalize) {
    result.tracking_lost = true;
    result.lost_cloud = filtered_point_cloud;
    result.tracking_range_data = tracking_range_data;
  }
  return absl::make_unique<MatchingResult>(std::move(result));
}

std::unique_ptr<LocalSlam2D::InsertionResult> LocalSlam2D::ResumeMapping(
    const common::Time time, const transform::Rigid2d& local_pose,
    const sensor::RangeData& tracking_range_data) {
  CHECK(extrapolator_ != nullptr);
  suspended_ = false;
  extrapolator_->AddPose(time, local_pose);
  has_range_pose_ = true;
  const sensor::PointCloud filtered = sensor::AdaptiveVoxelFilter(
      tracking_range_data.returns, options_.adaptive_voxel_filter_options());
  const sensor::RangeData range_data_in_local = TransformRangeData(
      tracking_range_data, transform::Embed3D(local_pose.cast<float>()));
  LOG(INFO) << "Relocalization stayed at the held pose; mapping resumes";
  return InsertIntoSubmap(time, range_data_in_local, filtered, local_pose);
}

void LocalSlam2D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    pending_odometry_data_.push_back(odometry_data);
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
  // Seed earlier than the first beam so motion-compensated LaserScan points
  // (relative times < 0) are not dropped as "still initializing".
  extrapolator_->AddPose(time - common::FromSeconds(0.25),
                         transform::Rigid2d());
  for (const auto& odometry_data : pending_odometry_data_) {
    extrapolator_->AddOdometryData(odometry_data);
  }
  pending_odometry_data_.clear();
}

}  // namespace mapping
}  // namespace cartographer
