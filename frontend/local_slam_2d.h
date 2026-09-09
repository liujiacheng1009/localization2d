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

 private:
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& tracking_range_data);
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
      const sensor::PointCloud& filtered_point_cloud);

  // Lazily constructs a PoseExtrapolator.
  void InitializeExtrapolator(common::Time time);

  const LocalTrajectoryBuilderOptions2D options_;
  ActiveSubmaps2D active_submaps_;

  MotionFilter motion_filter_;
  scan_matching::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;

  std::unique_ptr<PoseExtrapolator> extrapolator_;

  int num_accumulated_ = 0;
  sensor::RangeData accumulated_range_data_;

  const std::string expected_range_sensor_id_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRAJECTORY_LOCAL_SLAM_2D_H_
