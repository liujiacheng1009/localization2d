// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef CARTOGRAPHER_BACKEND_GTSAM_POSE_OPTIMIZER_2D_H_
#define CARTOGRAPHER_BACKEND_GTSAM_POSE_OPTIMIZER_2D_H_

#include "cartographer/backend/pose_optimizer_2d.h"

namespace cartographer {
namespace mapping {
namespace optimization {

// Batch SPA on a NonlinearFactorGraph (same trigger cadence as Ceres).
class GtsamBatchPoseOptimizer2D : public PoseOptimizer2D {
 public:
  explicit GtsamBatchPoseOptimizer2D(const OptimizationProblemOptions& options);
  ~GtsamBatchPoseOptimizer2D() override = default;

  void SetMaxNumIterations(int32 max_num_iterations) override;
  void Solve(const std::vector<Constraint>& constraints,
             const std::map<int, TrajectoryState>& trajectories_state) override;

 private:
  int max_num_iterations_;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_BACKEND_GTSAM_POSE_OPTIMIZER_2D_H_
