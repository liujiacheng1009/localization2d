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

#ifndef CARTOGRAPHER_TRAJECTORY_LOCAL_SLAM_2D_H_
#define CARTOGRAPHER_TRAJECTORY_LOCAL_SLAM_2D_H_

#include <atomic>
#include <deque>
#include <memory>
#include <string>

#include "cartographer/foundation/transform.h"
#include "cartographer/mapping/submap_2d.h"
#include "cartographer/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/frontend/motion_filter.h"
#include "cartographer/frontend/pose_extrapolator.h"
#include "cartographer/application/slam_options.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/foundation/voxel_filter.h"
#include "cartographer/foundation/sensor_data.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// TODO(gaschler): Add test for this class similar to the 3D test.
class LocalSlam2D {
 public:
  struct InsertionResult {
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
  };
  struct MatchingResult {
    common::Time time;
    transform::Rigid2d local_pose;
    sensor::RangeData range_data_in_local;
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;
    // The chassis is being carried, or it was tilted and is level again.
    // The returned pose is the last accepted one and nothing was inserted.
    // A global search may run. An in-place spin is not this case.
    bool tracking_lost = false;
    sensor::PointCloud lost_cloud;
    sensor::RangeData tracking_range_data;
  };

  explicit LocalSlam2D(
      const LocalTrajectoryBuilderOptions2D& options,
      std::string expected_range_sensor_id);
  ~LocalSlam2D();

  LocalSlam2D(const LocalSlam2D&) = delete;
  LocalSlam2D& operator=(const LocalSlam2D&) = delete;

  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'. Range data must be approximately horizontal
  // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  // Latest IMU tilt of the chassis, in radians from level. Zero until an
  // IMU sample arrives. Scan matching reads this on the mapping thread.
  void SetChassisTilt(double tilt_radians);
  bool IsTracking() const { return !suspended_; }
  // Jump to a pose found in a finished submap and insert there, instead of
  // painting the scan into the submap the robot was carried away from.
  std::unique_ptr<InsertionResult> CommitRelocalizedPose(
      common::Time time, const transform::Rigid2d& local_pose,
      const sensor::RangeData& tracking_range_data);
  // The search found the held pose again. Keep the current submap and insert.
  std::unique_ptr<InsertionResult> ResumeMapping(
      common::Time time, const transform::Rigid2d& local_pose,
      const sensor::RangeData& tracking_range_data);

 private:
  std::unique_ptr<MatchingResult> HoldLastPose(
      common::Time time, const sensor::RangeData& tracking_range_data,
      const sensor::PointCloud& filtered_point_cloud, bool relocalize);
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& tracking_range_data,
      double destort_yaw, double peak_segment_yaw,
      const sensor::PointCloud& early_tracking_hits,
      const sensor::PointCloud& late_tracking_hits);
  bool HalfScanPosesDisagree(const transform::Rigid2d& pose_estimate,
                             const sensor::PointCloud& early_tracking_hits,
                             const sensor::PointCloud& late_tracking_hits,
                             double* half_yaw, double* half_translation) const;
  bool HalfScanArmed(common::Time time, double odom_interval_yaw,
                     double matched_interval_yaw, double abs_destort_yaw,
                     double peak_segment_yaw);
  sensor::RangeData TransformToCurrentTrackingFrameAndFilter(
      const transform::Rigid3f& transform_to_current_tracking_frame,
      const sensor::RangeData& range_data) const;
  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_local,
      const sensor::PointCloud& filtered_point_cloud,
      const transform::Rigid2d& pose_estimate);

  // Scan matches 'filtered_point_cloud' and returns the
  // observed pose, or nullptr on failure.
  std::unique_ptr<transform::Rigid2d> ScanMatch(
      common::Time time, const transform::Rigid2d& pose_prediction,
      const sensor::PointCloud& filtered_point_cloud,
      double* correlative_score);

  // Lazily constructs a PoseExtrapolator.
  void InitializeExtrapolator(common::Time time);

  const LocalTrajectoryBuilderOptions2D options_;
  ActiveSubmaps2D active_submaps_;

  MotionFilter motion_filter_;
  scan_matching::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;
  // Narrow-window correlative matcher used only for post-turn half-scan checks.
  scan_matching::RealTimeCorrelativeScanMatcher2D
      half_scan_correlative_scan_matcher_;
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;

  std::unique_ptr<PoseExtrapolator> extrapolator_;
  std::deque<sensor::OdometryData> pending_odometry_data_;
  bool has_range_pose_ = false;
  // Latched while the chassis is tilted or lifted. Cleared only after a
  // level scan relocalizes, either by staying or by jumping.
  bool suspended_ = false;
  std::atomic<double> chassis_tilt_{0.};
  common::Time tilt_high_since_ = common::Time::min();
  // Half-scan runs only while time <= this (armed by recent turn signals).
  common::Time half_scan_armed_until_ = common::Time::min();

  int num_accumulated_ = 0;
  sensor::RangeData accumulated_range_data_;
  // Relative beam times (seconds, <= 0) parallel to accumulated returns only.
  std::vector<float> accumulated_hit_times_;

  const std::string expected_range_sensor_id_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRAJECTORY_LOCAL_SLAM_2D_H_
