// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef CARTOGRAPHER_BACKEND_GTSAM_ISAM2_POSE_OPTIMIZER_2D_H_
#define CARTOGRAPHER_BACKEND_GTSAM_ISAM2_POSE_OPTIMIZER_2D_H_

#include <set>
#include <utility>

#include "cartographer/backend/pose_optimizer_2d.h"
#include "gtsam/nonlinear/ISAM2.h"

namespace cartographer {
namespace mapping {
namespace optimization {

// Incremental SPA via GTSAM ISAM2 (same optimize_every_n_nodes trigger).
class GtsamIsam2PoseOptimizer2D : public PoseOptimizer2D {
 public:
  explicit GtsamIsam2PoseOptimizer2D(const OptimizationProblemOptions& options);
  ~GtsamIsam2PoseOptimizer2D() override = default;

  void TrimTrajectoryNode(const NodeId& node_id) override;
  void TrimSubmap(const SubmapId& submap_id) override;

  void SetMaxNumIterations(int32 max_num_iterations) override;
  void Solve(const std::vector<Constraint>& constraints,
             const std::map<int, TrajectoryState>& trajectories_state) override;

 private:
  void ResetIsam2();

  gtsam::ISAM2 isam2_;
  gtsam::ISAM2Params isam2_params_;
  std::set<SubmapId> inserted_submaps_;
  std::set<NodeId> inserted_nodes_;
  int constraints_fed_ = 0;
  // Consecutive node edges already added: (first_node, second_node).
  std::set<std::pair<NodeId, NodeId>> sequential_edges_;
  bool first_submap_anchored_ = false;
  int extra_update_passes_ = 0;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_BACKEND_GTSAM_ISAM2_POSE_OPTIMIZER_2D_H_
