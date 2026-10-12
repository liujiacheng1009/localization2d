// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef CARTOGRAPHER_BACKEND_CERES_POSE_OPTIMIZER_2D_H_
#define CARTOGRAPHER_BACKEND_CERES_POSE_OPTIMIZER_2D_H_

#include "cartographer/backend/pose_optimizer_2d.h"

namespace cartographer {
namespace mapping {
namespace optimization {

class CeresPoseOptimizer2D : public PoseOptimizer2D {
 public:
  explicit CeresPoseOptimizer2D(const OptimizationProblemOptions& options);
  ~CeresPoseOptimizer2D() override = default;

  void SetMaxNumIterations(int32 max_num_iterations) override;
  void Solve(const std::vector<Constraint>& constraints,
             const std::map<int, TrajectoryState>& trajectories_state) override;
};

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_BACKEND_CERES_POSE_OPTIMIZER_2D_H_
