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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <vector>

#include "cartographer/application/config.h"
#include "cartographer/application/slam_options.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/mapping/grid_2d.h"
#include "cartographer/scan_matching/correlative_scan_matcher_2d.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

RealTimeCorrelativeScanMatcherOptions2D
CreateRealTimeCorrelativeScanMatcherOptions2D(
    common::ParameterDictionary* parameter_dictionary);

// Exhaustive correlative search in a small window around the predicted pose.
// Used by local SLAM before Ceres so yaw can leave the odometry prior.
class RealTimeCorrelativeScanMatcher2D {
 public:
  explicit RealTimeCorrelativeScanMatcher2D(
      const RealTimeCorrelativeScanMatcherOptions2D& options);

  RealTimeCorrelativeScanMatcher2D(const RealTimeCorrelativeScanMatcher2D&) =
      delete;
  RealTimeCorrelativeScanMatcher2D& operator=(
      const RealTimeCorrelativeScanMatcher2D&) = delete;

  double Match(const transform::Rigid2d& initial_pose_estimate,
               const sensor::PointCloud& point_cloud, const Grid2D& grid,
               transform::Rigid2d* pose_estimate) const;

 private:
  std::vector<Candidate2D> GenerateExhaustiveSearchCandidates(
      const SearchParameters& search_parameters) const;
  void ScoreCandidates(const Grid2D& grid,
                       const std::vector<DiscreteScan2D>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate2D>* candidates) const;

  const RealTimeCorrelativeScanMatcherOptions2D options_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H_
