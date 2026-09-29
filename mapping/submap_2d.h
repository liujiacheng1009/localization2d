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

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/application/config.h"
#include "cartographer/mapping/grid_2d.h"
#include "cartographer/mapping/grid_2d.h"
#include "cartographer/application/slam_options.h"

#ifndef CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_

#include <utility>
#include <vector>

#include "cartographer/application/config.h"
#include "cartographer/foundation/time.h"
#include "cartographer/mapping/grid_2d.h"
#include "cartographer/mapping/grid_2d.h"
#include "cartographer/application/slam_options.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/foundation/sensor_data.h"

namespace cartographer {
namespace mapping {

ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
    common::ParameterDictionary* parameter_dictionary);
RangeDataInserterOptions CreateRangeDataInserterOptions(
    common::ParameterDictionary* parameter_dictionary);

class ProbabilityGridRangeDataInserter2D {
 public:
  explicit ProbabilityGridRangeDataInserter2D(
      const ProbabilityGridRangeDataInserterOptions2D& options);

  ProbabilityGridRangeDataInserter2D(
      const ProbabilityGridRangeDataInserter2D&) = delete;
  ProbabilityGridRangeDataInserter2D& operator=(
      const ProbabilityGridRangeDataInserter2D&) = delete;

  // Inserts 'range_data' into 'probability_grid'.
  void Insert(const sensor::RangeData& range_data,
              ProbabilityGrid* probability_grid) const;

 private:
  const ProbabilityGridRangeDataInserterOptions2D options_;
  const std::vector<uint16> hit_table_;
  const std::vector<uint16> miss_table_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_RANGE_DATA_INSERTER_2D_PROBABILITY_GRID_H_
#include "cartographer/mapping/submaps.h"
#include "cartographer/backend/trajectory_node.h"
#include "cartographer/mapping/grid_2d.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/foundation/transform.h"

namespace cartographer {
namespace mapping {

SubmapsOptions2D CreateSubmapsOptions2D(
    common::ParameterDictionary* parameter_dictionary);

class Submap2D : public Submap {
 public:
  Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid,
           ValueConversionTables* conversion_tables);
  Submap2D(const transform::Rigid2d& local_pose, int num_range_data,
           bool finished, std::unique_ptr<Grid2D> grid,
           ValueConversionTables* conversion_tables);

  void ToSubmapTextureResponse(
      const transform::Rigid2d& global_submap_pose,
      SubmapTextureResponse* response) const override;

  const Grid2D* grid() const { return grid_.get(); }

  // Insert 'range_data' into this submap using 'range_data_inserter'. The
  // submap must not be finished yet.
  void InsertRangeData(const sensor::RangeData& range_data,
                       const ProbabilityGridRangeDataInserter2D* range_data_inserter);
  void Finish();

 private:
  std::unique_ptr<Grid2D> grid_;
  ValueConversionTables* conversion_tables_;
};

// The first active submap will be created on the insertion of the first range
// data. Except during this initialization when no or only one single submap
// exists, there are always two submaps into which range data is inserted: an
// old submap that is used for matching, and a new one, which will be used for
// matching next, that is being initialized.
//
// Once a certain number of range data have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover, a
// "new" submap gets created. The "old" submap is forgotten by this object.
class ActiveSubmaps2D {
 public:
  explicit ActiveSubmaps2D(const SubmapsOptions2D& options);

  ActiveSubmaps2D(const ActiveSubmaps2D&) = delete;
  ActiveSubmaps2D& operator=(const ActiveSubmaps2D&) = delete;

  // Inserts 'range_data' into the Submap collection.
  std::vector<std::shared_ptr<const Submap2D>> InsertRangeData(
      const sensor::RangeData& range_data);

  std::vector<std::shared_ptr<const Submap2D>> submaps() const;
  // Drop the active pair and start one empty submap at 'origin'. The previous
  // grids stay alive through the backend and are not written again.
  void ReplaceWithNewOrigin(const Eigen::Vector2f& origin);

 private:
  std::unique_ptr<ProbabilityGrid> CreateGrid(const Eigen::Vector2f& origin);
  void FinishSubmap();
  void AddSubmap(const Eigen::Vector2f& origin);

  const SubmapsOptions2D options_;
  std::vector<std::shared_ptr<Submap2D>> submaps_;
  std::unique_ptr<ProbabilityGridRangeDataInserter2D> range_data_inserter_;
  ValueConversionTables conversion_tables_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAP_2D_H_
