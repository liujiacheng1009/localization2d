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

  // Match-quality gate: a wrong local minimum still returns a pose. When the
  // cloud no longer agrees with the active submap (and/or correlative score
  // is low), do not keep integrating that pose — hold and ask for Relocalize.
  const double min_agree = options_.tracking_min_match_agree_ratio();
  const double min_score = options_.tracking_min_correlative_score();
  const bool quality_enabled = min_agree > 0. || min_score > 0.;
  const double agree_ratio =
      quality_enabled ? MatchAgreeRatio(*pose_estimate_2d, filtered_point_cloud)
                      : 1.;
  const int bad_frames = options_.tracking_bad_match_frames() > 0
                             ? options_.tracking_bad_match_frames()
                             : 3;
  const bool agree_bad = min_agree > 0. && agree_ratio < min_agree;
  // Optional score reinforcement: only when agree is already below the floor.
  const bool score_bad = min_score > 0. && correlative_score >= 0. &&
                         correlative_score < min_score && agree_bad;
  if (has_range_pose_ && quality_enabled && agree_bad) {
    ++bad_match_streak_;
    LOG_EVERY_N(WARNING, 5)
        << "Poor scan match agree=" << agree_ratio << " score="
        << correlative_score << " streak=" << bad_match_streak_
        << (score_bad ? " (low score)" : "");
    if (bad_match_streak_ >= bad_frames) {
      suspended_ = true;
      LOG(WARNING) << "Tracking lost on match quality; holding pose for "
                      "relocalization agree="
                   << agree_ratio << " score=" << correlative_score;
      return HoldLastPose(time, tracking_range_data, filtered_point_cloud,
                          /*relocalize=*/true);
    }
    // Soft: trust wheels this frame, do not paint, do not latch LOST yet.
    extrapolator_->AddPose(time, pose_prediction);
    has_range_pose_ = true;
    sensor::RangeData held_range = TransformRangeData(
        tracking_range_data,
        transform::Embed3D(pose_prediction.cast<float>()));
    MatchingResult result{time, pose_prediction, std::move(held_range),
                          nullptr};
    return absl::make_unique<MatchingResult>(std::move(result));
  }
  bad_match_streak_ = 0;

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
  extrapolator_->AddPose(time, *pose_estimate_2d);
  has_range_pose_ = true;

  sensor::RangeData range_data_in_local =
      TransformRangeData(tracking_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  // Classify before the insert decision: point drops feed the ratio skip that
  // blocks bent sweeps from painting multilayer walls with the remaining hits.
  sensor::PointCloud rejected_insert_hits;
  const sensor::RangeData insert_range =
      FilterFarDisagreeingInsertHits(range_data_in_local, &rejected_insert_hits);
  const bool far_disagree_skip =
      FarDisagreeRatioSkipInsert(range_data_in_local, rejected_insert_hits);
  const bool skip_insert = gap_skip || turning_skip || destort_skip ||
                           peak_segment_skip || half_scan_skip ||
                           far_disagree_skip;

  std::unique_ptr<InsertionResult> insertion_result;
  if (skip_insert) {
    LOG(INFO) << "Skipping submap insert gap=" << gap_seconds
              << "s odom_yaw=" << odom_interval_yaw
              << " matched_yaw=" << matched_interval_yaw
              << " destort_yaw=" << abs_destort_yaw
              << " peak_segment_yaw=" << peak_segment_yaw
              << " half_armed=" << half_scan_armed
              << " half_yaw=" << half_yaw
              << " half_translation=" << half_translation
              << " far_disagree_skip=" << far_disagree_skip
              << " far_rejected=" << rejected_insert_hits.size();
  } else {
    insertion_result = InsertIntoSubmap(
        time, insert_range, filtered_point_cloud, *pose_estimate_2d,
        &rejected_insert_hits, /*apply_far_disagree_filter=*/false);
  }

  MatchingResult result{time, *pose_estimate_2d, std::move(range_data_in_local),
                        std::move(insertion_result)};
  result.rejected_insert_hits = std::move(rejected_insert_hits);
  return absl::make_unique<MatchingResult>(std::move(result));
}

sensor::RangeData LocalSlam2D::FilterFarDisagreeingInsertHits(
    const sensor::RangeData& range_data_in_local,
    sensor::PointCloud* rejected_hits) const {
  CHECK(rejected_hits != nullptr);
  *rejected_hits = sensor::PointCloud();
  const double insert_max = options_.skip_insert_max_range_meters();
  const double min_range = options_.skip_insert_far_disagree_min_range_meters();
  const double base = options_.skip_insert_far_disagree_base_meters();
  const double per_meter = options_.skip_insert_far_disagree_per_meter();
  const bool disagree_enabled =
      base > 0. && min_range > 0. && !active_submaps_.submaps().empty();
  if (insert_max <= 0. && !disagree_enabled) {
    return range_data_in_local;
  }
  const Grid2D* grid =
      disagree_enabled ? active_submaps_.submaps().front()->grid() : nullptr;
  const double resolution = grid != nullptr ? grid->limits().resolution() : 0.;
  // Correspondence cost below this ≈ occupied (probability > ~0.52).
  constexpr float kOccupiedCorrespondence = 0.48f;

  sensor::RangeData filtered = range_data_in_local;
  filtered.returns = sensor::PointCloud();
  const Eigen::Vector2f origin = range_data_in_local.origin.head<2>();

  for (const sensor::RangefinderPoint& hit : range_data_in_local.returns) {
    const Eigen::Vector2f p = hit.position.head<2>();
    const float range = (p - origin).norm();
    // Hard insert horizon: drop flying far returns (do not paint, do not
    // cast a long miss that would erode nearer walls).
    if (insert_max > 0. && range > insert_max) {
      rejected_hits->push_back(hit);
      continue;
    }
    if (!disagree_enabled || range < min_range || range < 1e-3f) {
      filtered.returns.push_back(hit);
      continue;
    }
    const double threshold = base + per_meter * static_cast<double>(range);
    // Ray residual vs the *first* occupied cell. Neighborhood search would
    // "agree" with an already-painted ghost wall at the bad hit.
    // Past the first wall inside the ghost band → drop (parallel multilayer).
    // Larger gaps stay (doorway → next room). Short-of-wall ghosts in-band drop;
    // much closer returns stay (real foreground objects / dynamics).
    constexpr double kMaxGhostBandMeters = 0.45;
    const Eigen::Vector2f dir = (p - origin) / range;
    const double walk =
        std::min(static_cast<double>(range) + kMaxGhostBandMeters, 12.0);
    const int steps =
        std::max(1, static_cast<int>(std::ceil(walk / resolution)));
    bool found_wall = false;
    double wall_range = 0.;
    for (int i = 1; i <= steps; ++i) {
      const double d = static_cast<double>(i) * resolution;
      const Eigen::Vector2f q = origin + dir * static_cast<float>(d);
      const Eigen::Array2i idx = grid->limits().GetCellIndex(q);
      if (!grid->IsKnown(idx)) continue;
      if (grid->GetCorrespondenceCost(idx) > kOccupiedCorrespondence) {
        continue;
      }
      found_wall = true;
      wall_range = d;
      break;
    }
    if (!found_wall) {
      filtered.returns.push_back(hit);
      continue;
    }
    const double delta = static_cast<double>(range) - wall_range;
    const bool agrees = std::abs(delta) <= threshold;
    const bool behind_ghost =
        delta > threshold && delta <= kMaxGhostBandMeters;
    const bool short_ghost =
        delta < -threshold && -delta <= kMaxGhostBandMeters;
    if (agrees) {
      filtered.returns.push_back(hit);
    } else if (behind_ghost || short_ghost) {
      rejected_hits->push_back(hit);
    } else {
      filtered.returns.push_back(hit);
    }
  }
  return filtered;
}

double LocalSlam2D::MatchAgreeRatio(
    const transform::Rigid2d& local_pose,
    const sensor::PointCloud& tracking_hits) const {
  if (active_submaps_.submaps().empty() || tracking_hits.empty()) {
    return 1.;
  }
  const Grid2D& grid = *active_submaps_.submaps().front()->grid();
  const double resolution = grid.limits().resolution();
  constexpr float kOccupiedCorrespondence = 0.48f;
  // Tight radius: a 0.12 m disk still "agrees" with thick/multilayer walls
  // when the pose is visibly wrong. ~2 cells @ 3 cm catches true matches.
  constexpr double kAgreeRadius = 0.06;
  const int rad =
      std::max(1, static_cast<int>(std::ceil(kAgreeRadius / resolution)));
  int considered = 0;
  int agrees = 0;
  for (const sensor::RangefinderPoint& hit : tracking_hits) {
    const float range = hit.position.head<2>().norm();
    if (range < 0.5f || range > 5.0f) continue;
    ++considered;
    const Eigen::Vector2f local =
        (local_pose.cast<float>() * hit.position.head<2>());
    const Eigen::Array2i center = grid.limits().GetCellIndex(local);
    bool ok = false;
    for (int dy = -rad; dy <= rad && !ok; ++dy) {
      for (int dx = -rad; dx <= rad; ++dx) {
        if (dx * dx + dy * dy > rad * rad + rad) continue;
        const Eigen::Array2i idx(center.x() + dx, center.y() + dy);
        if (!grid.IsKnown(idx)) continue;
        if (grid.GetCorrespondenceCost(idx) <= kOccupiedCorrespondence) {
          ok = true;
          break;
        }
      }
    }
    if (ok) ++agrees;
  }
  if (considered < 25) return 1.;
  return static_cast<double>(agrees) / static_cast<double>(considered);
}

bool LocalSlam2D::FarDisagreeRatioSkipInsert(
    const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& rejected_hits) const {
  const double ratio = options_.skip_insert_far_disagree_ratio();
  const double min_range = options_.skip_insert_far_disagree_min_range_meters();
  const double insert_max = options_.skip_insert_max_range_meters();
  if (ratio <= 0. || min_range <= 0. ||
      options_.skip_insert_far_disagree_base_meters() <= 0.) {
    return false;
  }
  const Eigen::Vector2f origin = range_data_in_local.origin.head<2>();
  auto in_ratio_band = [&](const float range) {
    if (range < min_range) return false;
    if (insert_max > 0. && range > insert_max) return false;
    return true;
  };
  int far = 0;
  for (const sensor::RangefinderPoint& hit : range_data_in_local.returns) {
    if (in_ratio_band((hit.position.head<2>() - origin).norm())) {
      ++far;
    }
  }
  if (far < 25) return false;
  int rejected_in_band = 0;
  for (const sensor::RangefinderPoint& hit : rejected_hits) {
    if (in_ratio_band((hit.position.head<2>() - origin).norm())) {
      ++rejected_in_band;
    }
  }
  return static_cast<double>(rejected_in_band) >= ratio * far;
}

std::unique_ptr<LocalSlam2D::InsertionResult>
LocalSlam2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_point_cloud,
    const transform::Rigid2d& pose_estimate,
    sensor::PointCloud* rejected_insert_hits,
    const bool apply_far_disagree_filter) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  sensor::PointCloud rejected_local;
  sensor::PointCloud* rejected =
      rejected_insert_hits != nullptr ? rejected_insert_hits : &rejected_local;
  const sensor::RangeData insert_range =
      apply_far_disagree_filter
          ? FilterFarDisagreeingInsertHits(range_data_in_local, rejected)
          : range_data_in_local;
  if (insert_range.returns.empty()) {
    return nullptr;
  }
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(insert_range);
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
  bad_match_streak_ = 0;
  extrapolator_->AddPose(time, local_pose);
  has_range_pose_ = true;
  active_submaps_.ReplaceWithNewOrigin(local_pose.translation().cast<float>());
  const sensor::PointCloud filtered = sensor::AdaptiveVoxelFilter(
      tracking_range_data.returns, options_.adaptive_voxel_filter_options());
  const sensor::RangeData range_data_in_local = TransformRangeData(
      tracking_range_data, transform::Embed3D(local_pose.cast<float>()));
  LOG(INFO) << "Relocalized local pose to (" << local_pose.translation().x()
            << ", " << local_pose.translation().y() << ")";
  // Reloc confirm used to paint unconditionally and bypass skip_insert_*.
  // Require the cloud to agree with the new active submap before inserting.
  double min_agree = options_.tracking_min_match_agree_ratio();
  if (min_agree <= 0.) {
    min_agree = 0.35;
  }
  const double agree = MatchAgreeRatio(local_pose, filtered);
  if (agree < min_agree) {
    LOG(WARNING) << "Relocalization jump pose accepted but skip insert: agree="
                 << agree << " min=" << min_agree;
    return nullptr;
  }
  return InsertIntoSubmap(time, range_data_in_local, filtered, local_pose,
                          nullptr);
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
    const sensor::RangeData& /*tracking_range_data*/) {
  CHECK(extrapolator_ != nullptr);
  suspended_ = false;
  bad_match_streak_ = 0;
  extrapolator_->AddPose(time, local_pose);
  has_range_pose_ = true;
  // Stay means the held pose is still usable. Do not paint here: that used to
  // bypass turning/destort/far-disagree gates and write bent scans (e.g. f2358).
  LOG(INFO) << "Relocalization stayed at the held pose; mapping resumes "
               "without insert";
  return nullptr;
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
