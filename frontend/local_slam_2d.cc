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
#include <vector>

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

// Largest |yaw| of TrackingMotion over any intra-scan window of
// `window_seconds`. Partial bend often comes from a short fast turn inside
// the sweep, while first→last destort_yaw stays modest.
double PeakSegmentYaw(const PoseExtrapolator& extrapolator,
                      const common::Time end_time,
                      const sensor::TimedPointCloud& ranges,
                      const double window_seconds) {
  if (ranges.size() < 2 || window_seconds <= 0.) {
    return 0.;
  }
  const size_t step =
      std::max<size_t>(1, ranges.size() / 64);  // ~64 samples across the sweep
  std::vector<size_t> indices;
  indices.reserve(64);
  for (size_t i = 0; i < ranges.size(); i += step) {
    indices.push_back(i);
  }
  if (indices.back() != ranges.size() - 1) {
    indices.push_back(ranges.size() - 1);
  }

  double peak = 0.;
  size_t j = 0;
  for (size_t a = 0; a < indices.size(); ++a) {
    const float rel_a = ranges[indices[a]].time;
    if (j < a) {
      j = a;
    }
    while (j + 1 < indices.size() &&
           ranges[indices[j]].time - rel_a < window_seconds) {
      ++j;
    }
    if (j >= indices.size() ||
        ranges[indices[j]].time - rel_a < window_seconds) {
      break;
    }
    const common::Time t_a = end_time + common::FromSeconds(rel_a);
    const common::Time t_b =
        end_time + common::FromSeconds(ranges[indices[j]].time);
    peak = std::max(
        peak, std::abs(transform::Yaw(extrapolator.TrackingMotion(t_a, t_b))));
  }
  return peak;
}

RealTimeCorrelativeScanMatcherOptions2D HalfScanMatcherOptions(
    const LocalTrajectoryBuilderOptions2D& options) {
  RealTimeCorrelativeScanMatcherOptions2D half =
      options.real_time_correlative_scan_matcher_options();
  if (options.skip_insert_half_scan_linear_search_window() > 0.) {
    half.set_linear_search_window(
        options.skip_insert_half_scan_linear_search_window());
  }
  if (options.skip_insert_half_scan_angular_search_window() > 0.) {
    half.set_angular_search_window(
        options.skip_insert_half_scan_angular_search_window());
  }
  return half;
}

}  // namespace

LocalSlam2D::LocalSlam2D(
    const LocalTrajectoryBuilderOptions2D& options,
    std::string expected_range_sensor_id)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      half_scan_correlative_scan_matcher_(HalfScanMatcherOptions(options)),
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
    const sensor::PointCloud& filtered_point_cloud,
    double* const correlative_score) {
  CHECK(correlative_score != nullptr);
  *correlative_score = -1.;
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  transform::Rigid2d initial_ceres_pose = pose_prediction;
  if (options_.use_online_correlative_scan_matching()) {
    *correlative_score = real_time_correlative_scan_matcher_.Match(
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
  double destort_yaw = 0.;
  const double peak_window = options_.skip_insert_peak_segment_seconds();
  const double peak_segment_yaw =
      PeakSegmentYaw(*extrapolator_, time, range_data.ranges, peak_window);
  if (std::abs(range_data.ranges.front().time) > 1e-4) {
    const transform::Rigid2d first_in_end = extrapolator_->TrackingMotion(
        time + common::FromSeconds(range_data.ranges.front().time), time);
    destort_yaw = transform::Yaw(first_in_end);
    if (std::abs(destort_yaw) > 0.08 ||
        (peak_window > 0. && peak_segment_yaw > 0.05)) {
      LOG(INFO) << "Scan destortion yaw=" << destort_yaw << " rad over "
                << -range_data.ranges.front().time
                << " s peak_segment_yaw=" << peak_segment_yaw << " rad in "
                << peak_window << " s";
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
    accumulated_hit_times_.clear();
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
        accumulated_hit_times_.push_back(hit.time);
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
    const transform::Rigid3f to_tracking = range_data_poses.back().inverse();
    sensor::PointCloud early_tracking_hits;
    sensor::PointCloud late_tracking_hits;
    CHECK_EQ(accumulated_range_data_.returns.size(),
             accumulated_hit_times_.size());
    if (!accumulated_hit_times_.empty()) {
      std::vector<float> sorted_times = accumulated_hit_times_;
      std::nth_element(sorted_times.begin(),
                       sorted_times.begin() + sorted_times.size() / 2,
                       sorted_times.end());
      const float median_time = sorted_times[sorted_times.size() / 2];
      for (size_t i = 0; i < accumulated_range_data_.returns.size(); ++i) {
        const sensor::RangefinderPoint in_tracking =
            to_tracking * accumulated_range_data_.returns[i];
        if (accumulated_hit_times_[i] <= median_time) {
          early_tracking_hits.push_back(in_tracking);
        } else {
          late_tracking_hits.push_back(in_tracking);
        }
      }
      early_tracking_hits = sensor::VoxelFilter(
          early_tracking_hits, options_.voxel_filter_size());
      late_tracking_hits = sensor::VoxelFilter(
          late_tracking_hits, options_.voxel_filter_size());
    }
    accumulated_hit_times_.clear();
    return AddAccumulatedRangeData(
        time,
        TransformToCurrentTrackingFrameAndFilter(
            to_tracking, accumulated_range_data_),
        destort_yaw, peak_segment_yaw, early_tracking_hits,
        late_tracking_hits);
  }
  return nullptr;
}

bool LocalSlam2D::HalfScanArmed(const common::Time time,
                                const double odom_interval_yaw,
                                const double matched_interval_yaw,
                                const double abs_destort_yaw,
                                const double peak_segment_yaw) {
  const double after_s = options_.skip_insert_half_scan_after_turn_seconds();
  const double arm_yaw = options_.skip_insert_half_scan_arm_yaw_radians();
  // Disabled when either arming knob is unset.
  if (after_s <= 0. || arm_yaw <= 0.) {
    return false;
  }
  const double signal = std::max(std::max(odom_interval_yaw, matched_interval_yaw),
                                 std::max(abs_destort_yaw, peak_segment_yaw));
  if (signal > arm_yaw) {
    half_scan_armed_until_ = time + common::FromSeconds(after_s);
  }
  return time <= half_scan_armed_until_;
}

bool LocalSlam2D::HalfScanPosesDisagree(
    const transform::Rigid2d& pose_estimate,
    const sensor::PointCloud& early_tracking_hits,
    const sensor::PointCloud& late_tracking_hits, double* half_yaw,
    double* half_translation) const {
  CHECK(half_yaw != nullptr);
  CHECK(half_translation != nullptr);
  *half_yaw = 0.;
  *half_translation = 0.;
  const double yaw_limit = options_.skip_insert_half_scan_yaw_radians();
  const double translation_limit =
      options_.skip_insert_half_scan_translation_meters();
  if ((yaw_limit <= 0. && translation_limit <= 0.) ||
      early_tracking_hits.size() < 30 || late_tracking_hits.size() < 30 ||
      active_submaps_.submaps().empty()) {
    return false;
  }
  const Grid2D& grid = *active_submaps_.submaps().front()->grid();
  transform::Rigid2d pose_early = pose_estimate;
  transform::Rigid2d pose_late = pose_estimate;
  half_scan_correlative_scan_matcher_.Match(
      pose_estimate, early_tracking_hits, grid, &pose_early);
  half_scan_correlative_scan_matcher_.Match(
      pose_estimate, late_tracking_hits, grid, &pose_late);
  const transform::Rigid2d late_from_early = pose_early.inverse() * pose_late;
  *half_yaw = std::abs(transform::Yaw(late_from_early));
  *half_translation = late_from_early.translation().norm();
  // OR on purpose: requiring both under-triggers once multilayer ghosts are
  // already in the submap (both halves lock onto the same wrong wall).
  return (yaw_limit > 0. && *half_yaw > yaw_limit) ||
         (translation_limit > 0. && *half_translation > translation_limit);
}

std::unique_ptr<LocalSlam2D::MatchingResult>
LocalSlam2D::AddAccumulatedRangeData(
    const common::Time time, const sensor::RangeData& tracking_range_data,
    const double destort_yaw, const double peak_segment_yaw,
    const sensor::PointCloud& early_tracking_hits,
    const sensor::PointCloud& late_tracking_hits) {
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

  double correlative_score = -1.;
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction, filtered_point_cloud,
                &correlative_score);
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  // The local correlative window can occasionally choose a nearby wall
  // fragment. Compare that displacement with wheel odometry before accepting
  // it; only a high-confidence, bounded laser match may override a stale
  // odometry sample. A low-confidence disagreement uses the prediction for
  // this frame and never latches a localization hold.
  if (has_range_pose_) {
    const transform::Rigid2d from_last =
        extrapolator_->GetLastPose().inverse() * pose_prediction;
    const transform::Rigid2d from_prediction =
        pose_prediction.inverse() * *pose_estimate_2d;
    const bool translation_still = from_last.translation().norm() < 0.05;
    const bool yaw_still = std::abs(transform::Yaw(from_last)) < 0.08;
    const bool walked = from_prediction.translation().norm() > 0.12;
    if (walked && translation_still && yaw_still) {
      // Prefer a well-supported laser match over a stale wheel sample. The
      // correlative score is normalized to [0, 1] and is only available when
      // online correlative matching is enabled. A high score plus a bounded
      // displacement is evidence that the robot really moved; otherwise use
      // the odometry prediction for this frame without latching suspension.
      constexpr double kHighConfidenceScore = 0.65;
      constexpr double kMaximumTrustedScanJump = 0.50;
      const bool trusted_scan = correlative_score >= kHighConfidenceScore &&
          from_prediction.translation().norm() <= kMaximumTrustedScanJump;
      if (trusted_scan) {
        LOG_EVERY_N(INFO, 20)
            << "Trusted laser motion despite stale odometry: score="
            << correlative_score << " scan_delta="
            << from_prediction.translation().norm();
      } else {
        LOG_EVERY_N(WARNING, 20)
            << "Scan/odometry disagreement; using odometry prediction: score="
            << correlative_score << " scan_delta="
            << from_prediction.translation().norm();
        const transform::Rigid2d held = pose_prediction;
        extrapolator_->AddPose(time, held);
        has_range_pose_ = true;
        sensor::RangeData held_range = TransformRangeData(
            tracking_range_data, transform::Embed3D(held.cast<float>()));
        MatchingResult result{time, held, std::move(held_range), nullptr};
        return absl::make_unique<MatchingResult>(std::move(result));
      }
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
  const double destort_limit = options_.skip_insert_destort_yaw_radians();
  const double abs_destort_yaw = std::abs(destort_yaw);
  const bool destort_skip =
      destort_limit > 0. && abs_destort_yaw > destort_limit;
  const double peak_yaw_limit =
      options_.skip_insert_peak_segment_yaw_radians();
  const bool peak_segment_skip =
      options_.skip_insert_peak_segment_seconds() > 0. &&
      peak_yaw_limit > 0. && peak_segment_yaw > peak_yaw_limit;
  double half_yaw = 0.;
  double half_translation = 0.;
  const bool half_scan_armed =
      HalfScanArmed(time, odom_interval_yaw, matched_interval_yaw,
                    abs_destort_yaw, peak_segment_yaw);
  const bool half_scan_skip =
      half_scan_armed &&
      HalfScanPosesDisagree(*pose_estimate_2d, early_tracking_hits,
                            late_tracking_hits, &half_yaw, &half_translation);
  const bool skip_insert = gap_skip || turning_skip || destort_skip ||
                           peak_segment_skip || half_scan_skip;
  extrapolator_->AddPose(time, *pose_estimate_2d);
  has_range_pose_ = true;

  sensor::RangeData range_data_in_local =
      TransformRangeData(tracking_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  std::unique_ptr<InsertionResult> insertion_result;
  if (skip_insert) {
    LOG(INFO) << "Skipping submap insert gap=" << gap_seconds
              << "s odom_yaw=" << odom_interval_yaw
              << " matched_yaw=" << matched_interval_yaw
              << " destort_yaw=" << abs_destort_yaw
              << " peak_segment_yaw=" << peak_segment_yaw
              << " half_armed=" << half_scan_armed
              << " half_yaw=" << half_yaw
              << " half_translation=" << half_translation;
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
