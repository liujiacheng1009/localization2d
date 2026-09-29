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

#include "cartographer/mapping/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/foundation/time.h"


#include <cstdlib>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/grid_2d.h"

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_TO_PIXEL_MASK_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_TO_PIXEL_MASK_H_

#include <vector>

#include "cartographer/foundation/transform.h"

namespace cartographer {
namespace mapping {

// Compute all pixels that contain some part of the line segment connecting
// 'scaled_begin' and 'scaled_end'. 'scaled_begin' and 'scaled_end' are scaled
// by 'subpixel_scale'. 'scaled_begin' and 'scaled_end' are expected to be
// greater than zero. Return values are in pixels and not scaled.
std::vector<Eigen::Array2i> RayToPixelMask(const Eigen::Array2i& scaled_begin,
                                           const Eigen::Array2i& scaled_end,
                                           int subpixel_scale);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_RAY_TO_PIXEL_MASK_H_


#include "Eigen/Dense"

namespace cartographer {
namespace mapping {
namespace {

bool isEqual(const Eigen::Array2i& lhs, const Eigen::Array2i& rhs) {
  return ((lhs - rhs).matrix().lpNorm<1>() == 0);
}
}  // namespace

// Compute all pixels that contain some part of the line segment connecting
// 'scaled_begin' and 'scaled_end'. 'scaled_begin' and 'scaled_end' are scaled
// by 'subpixel_scale'. 'scaled_begin' and 'scaled_end' are expected to be
// greater than zero. Return values are in pixels and not scaled.
std::vector<Eigen::Array2i> RayToPixelMask(const Eigen::Array2i& scaled_begin,
                                           const Eigen::Array2i& scaled_end,
                                           int subpixel_scale) {
  // For simplicity, we order 'scaled_begin' and 'scaled_end' by their x
  // coordinate.
  if (scaled_begin.x() > scaled_end.x()) {
    return RayToPixelMask(scaled_end, scaled_begin, subpixel_scale);
  }

  CHECK_GE(scaled_begin.x(), 0);
  CHECK_GE(scaled_begin.y(), 0);
  CHECK_GE(scaled_end.y(), 0);
  std::vector<Eigen::Array2i> pixel_mask;
  // Special case: We have to draw a vertical line in full pixels, as
  // 'scaled_begin' and 'scaled_end' have the same full pixel x coordinate.
  if (scaled_begin.x() / subpixel_scale == scaled_end.x() / subpixel_scale) {
    Eigen::Array2i current(
        scaled_begin.x() / subpixel_scale,
        std::min(scaled_begin.y(), scaled_end.y()) / subpixel_scale);
    pixel_mask.push_back(current);
    const int end_y =
        std::max(scaled_begin.y(), scaled_end.y()) / subpixel_scale;
    for (; current.y() <= end_y; ++current.y()) {
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    }
    return pixel_mask;
  }

  const int64 dx = scaled_end.x() - scaled_begin.x();
  const int64 dy = scaled_end.y() - scaled_begin.y();
  const int64 denominator = 2 * subpixel_scale * dx;

  // The current full pixel coordinates. We scaled_begin at 'scaled_begin'.
  Eigen::Array2i current = scaled_begin / subpixel_scale;
  pixel_mask.push_back(current);

  // To represent subpixel centers, we use a factor of 2 * 'subpixel_scale' in
  // the denominator.
  // +-+-+-+ -- 1 = (2 * subpixel_scale) / (2 * subpixel_scale)
  // | | | |
  // +-+-+-+
  // | | | |
  // +-+-+-+ -- top edge of first subpixel = 2 / (2 * subpixel_scale)
  // | | | | -- center of first subpixel = 1 / (2 * subpixel_scale)
  // +-+-+-+ -- 0 = 0 / (2 * subpixel_scale)

  // The center of the subpixel part of 'scaled_begin.y()' assuming the
  // 'denominator', i.e., sub_y / denominator is in (0, 1).
  int64 sub_y = (2 * (scaled_begin.y() % subpixel_scale) + 1) * dx;

  // The distance from the from 'scaled_begin' to the right pixel border, to be
  // divided by 2 * 'subpixel_scale'.
  const int first_pixel =
      2 * subpixel_scale - 2 * (scaled_begin.x() % subpixel_scale) - 1;
  // The same from the left pixel border to 'scaled_end'.
  const int last_pixel = 2 * (scaled_end.x() % subpixel_scale) + 1;

  // The full pixel x coordinate of 'scaled_end'.
  const int end_x = std::max(scaled_begin.x(), scaled_end.x()) / subpixel_scale;

  // Move from 'scaled_begin' to the next pixel border to the right.
  sub_y += dy * first_pixel;
  if (dy > 0) {
    while (true) {
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
      }
      ++current.x();
      if (sub_y == denominator) {
        sub_y -= denominator;
        ++current.y();
      }
      if (current.x() == end_x) {
        break;
      }
      // Move from one pixel border to the next.
      sub_y += dy * 2 * subpixel_scale;
    }
    // Move from the pixel border on the right to 'scaled_end'.
    sub_y += dy * last_pixel;
    if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), scaled_end.y() / subpixel_scale);
    return pixel_mask;
  }

  // Same for lines non-ascending in y coordinates.
  while (true) {
    if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
    }
    ++current.x();
    if (sub_y == 0) {
      sub_y += denominator;
      --current.y();
    }
    if (current.x() == end_x) {
      break;
    }
    sub_y += dy * 2 * subpixel_scale;
  }
  sub_y += dy * last_pixel;
  if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    if (!isEqual(pixel_mask.back(), current)) pixel_mask.push_back(current);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), scaled_end.y() / subpixel_scale);
  return pixel_mask;
}

}  // namespace mapping
}  // namespace cartographer
#include "cartographer/mapping/grid_2d.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;

void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());
  // Padding around bounding box to avoid numerical issues at cell boundaries.
  constexpr float kPadding = 1e-6f;
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    bounding_box.extend(hit.position.head<2>());
  }
  for (const sensor::RangefinderPoint& miss : range_data.misses) {
    bounding_box.extend(miss.position.head<2>());
  }
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());
}

void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space, ProbabilityGrid* probability_grid) {
  GrowAsNeeded(range_data, probability_grid);

  const MapLimits& limits = probability_grid->limits();
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin =
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }

  if (!insert_free_space) {
    return;
  }

  // Now add the misses.
  for (const Eigen::Array2i& end : ends) {
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
    std::vector<Eigen::Array2i> ray = RayToPixelMask(
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }
}
}  // namespace

ProbabilityGridRangeDataInserterOptions2D
CreateProbabilityGridRangeDataInserterOptions2D(
    common::ParameterDictionary* parameter_dictionary) {
  ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

RangeDataInserterOptions CreateRangeDataInserterOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  CHECK_EQ(parameter_dictionary->GetString("range_data_inserter_type"),
           "PROBABILITY_GRID_INSERTER_2D");
  RangeDataInserterOptions options;
  options.set_range_data_inserter_type(
      RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D);
  *options.mutable_probability_grid_range_data_inserter_options_2d() =
      CreateProbabilityGridRangeDataInserterOptions2D(
          parameter_dictionary
              ->GetDictionary("probability_grid_range_data_inserter")
              .get());
  return options;
}

ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability()))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability()))) {}

void ProbabilityGridRangeDataInserter2D::Insert(
    const sensor::RangeData& range_data,
    ProbabilityGrid* const probability_grid) const {
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(),
           probability_grid);
  probability_grid->FinishUpdate();
}

}  // namespace mapping
}  // namespace cartographer
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

SubmapsOptions2D CreateSubmapsOptions2D(
    common::ParameterDictionary* const parameter_dictionary) {
  SubmapsOptions2D options;
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  *options.mutable_grid_options_2d() = CreateGridOptions2D(
      parameter_dictionary->GetDictionary("grid_options_2d").get());
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());

  CHECK_GT(options.num_range_data(), 0);
  return options;
}

Submap2D::Submap2D(const Eigen::Vector2f& origin, std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::MakeRigid2Translation(origin.cast<double>())),
      conversion_tables_(conversion_tables) {
  grid_ = std::move(grid);
}

Submap2D::Submap2D(const transform::Rigid2d& local_pose, int num_range_data,
                   bool finished, std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables)
    : Submap(local_pose),
      grid_(std::move(grid)),
      conversion_tables_(conversion_tables) {
  set_num_range_data(num_range_data);
  set_insertion_finished(finished);
}

void Submap2D::ToSubmapTextureResponse(
    const transform::Rigid2d&,
    SubmapTextureResponse* const response) const {
  if (!grid_) return;
  response->submap_version = num_range_data();
  response->textures.emplace_back();
  SubmapTexture* const texture = &response->textures.back();
  grid()->DrawToSubmapTexture(texture, local_pose());
}

void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const ProbabilityGridRangeDataInserter2D* range_data_inserter) {
  CHECK(grid_);
  CHECK(!insertion_finished());
  range_data_inserter->Insert(range_data,
                              static_cast<ProbabilityGrid*>(grid_.get()));
  set_num_range_data(num_range_data() + 1);
}

void Submap2D::Finish() {
  CHECK(grid_);
  CHECK(!insertion_finished());
  grid_ = grid_->ComputeCroppedGrid();
  set_insertion_finished(true);
}

ActiveSubmaps2D::ActiveSubmaps2D(const SubmapsOptions2D& options)
    : options_(options),
      range_data_inserter_(
          absl::make_unique<ProbabilityGridRangeDataInserter2D>(
              options.range_data_inserter_options()
                  .probability_grid_range_data_inserter_options_2d())) {}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const {
  return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(),
                                                      submaps_.end());
}

void ActiveSubmaps2D::ReplaceWithNewOrigin(const Eigen::Vector2f& origin) {
  submaps_.clear();
  AddSubmap(origin);
}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
    const sensor::RangeData& range_data) {
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());
  }
  for (auto& submap : submaps_) {
    submap->InsertRangeData(range_data, range_data_inserter_.get());
  }
  if (submaps_.front()->num_range_data() == 2 * options_.num_range_data()) {
    submaps_.front()->Finish();
  }
  return submaps();
}

std::unique_ptr<ProbabilityGrid> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin) {
  constexpr int kInitialSubmapSize = 100;
  float resolution = options_.grid_options_2d().resolution();
  return absl::make_unique<ProbabilityGrid>(
      MapLimits(resolution,
                origin.cast<double>() + 0.5 * kInitialSubmapSize * resolution *
                                            Eigen::Vector2d::Ones(),
                CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
      &conversion_tables_);
}

void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  if (submaps_.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(submaps_.front()->insertion_finished());
    submaps_.erase(submaps_.begin());
  }
  submaps_.push_back(absl::make_unique<Submap2D>(
      origin,
      CreateGrid(origin),
      &conversion_tables_));
}

}  // namespace mapping
}  // namespace cartographer
