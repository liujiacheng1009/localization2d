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

#ifndef CARTOGRAPHER_BACKEND_POSE_OPTIMIZER_2D_H_
#define CARTOGRAPHER_BACKEND_POSE_OPTIMIZER_2D_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/application/slam_options.h"
#include "cartographer/backend/backend_types.h"
#include "cartographer/backend/map_by_id.h"
#include "cartographer/backend/trajectory_history.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/foundation/time.h"
#include "cartographer/foundation/transform.h"

namespace cartographer {
namespace mapping {
namespace optimization {

struct NodeSpec2D {
  common::Time time;
  transform::Rigid2d local_pose_2d;
  transform::Rigid2d global_pose_2d;
};

struct SubmapSpec2D {
  transform::Rigid2d global_pose;
};

// Per-Solve timing for backend comparison (ceres vs isam2).
struct SolveMetric {
  int index = 0;
  double wall_seconds = 0.;
  double cpu_seconds = 0.;
  int num_nodes = 0;
  int num_submaps = 0;
  int num_constraints = 0;
};

// Shared SPA problem state; concrete backends implement Solve().
class PoseOptimizer2D {
 public:
  using Constraint = ::cartographer::mapping::Constraint;

  virtual ~PoseOptimizer2D();

  PoseOptimizer2D(const PoseOptimizer2D&) = delete;
  PoseOptimizer2D& operator=(const PoseOptimizer2D&) = delete;

  void AddOdometryData(int trajectory_id,
                       const sensor::OdometryData& odometry_data);
  void AddTrajectoryNode(int trajectory_id, const NodeSpec2D& node_data);
  void InsertTrajectoryNode(const NodeId& node_id, const NodeSpec2D& node_data);
  virtual void TrimTrajectoryNode(const NodeId& node_id);
  void AddSubmap(int trajectory_id,
                 const transform::Rigid2d& global_submap_pose);
  void InsertSubmap(const SubmapId& submap_id,
                    const transform::Rigid2d& global_submap_pose);
  virtual void TrimSubmap(const SubmapId& submap_id);

  virtual void SetMaxNumIterations(int32 max_num_iterations) = 0;
  virtual void Solve(const std::vector<Constraint>& constraints,
                     const std::map<int, TrajectoryState>& trajectories_state) = 0;

  const MapById<NodeId, NodeSpec2D>& node_data() const { return node_data_; }
  const MapById<SubmapId, SubmapSpec2D>& submap_data() const {
    return submap_data_;
  }
  const sensor::MapByTime<sensor::OdometryData>& odometry_data() const {
    return odometry_data_;
  }
  const std::vector<SolveMetric>& solve_metrics() const {
    return solve_metrics_;
  }
  const std::string& backend_name() const { return backend_name_; }

 protected:
  PoseOptimizer2D(const OptimizationProblemOptions& options,
                  std::string backend_name);

  std::unique_ptr<transform::Rigid2d> InterpolateOdometry(
      int trajectory_id, common::Time time) const;
  std::unique_ptr<transform::Rigid2d> CalculateOdometryBetweenNodes(
      int trajectory_id, const NodeSpec2D& first_node_data,
      const NodeSpec2D& second_node_data) const;

  void RecordSolveMetric(double wall_seconds, double cpu_seconds,
                         int num_constraints);

  OptimizationProblemOptions options_;
  MapById<NodeId, NodeSpec2D> node_data_;
  MapById<SubmapId, SubmapSpec2D> submap_data_;
  sensor::MapByTime<sensor::OdometryData> odometry_data_;
  std::vector<SolveMetric> solve_metrics_;
  std::string backend_name_;
};

std::unique_ptr<PoseOptimizer2D> CreatePoseOptimizer2D(
    const PoseGraphOptions& pose_graph_options);

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_BACKEND_POSE_OPTIMIZER_2D_H_
