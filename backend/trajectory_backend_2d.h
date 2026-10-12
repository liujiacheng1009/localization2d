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

#ifndef CARTOGRAPHER_BACKEND_TRAJECTORY_BACKEND_2D_H_
#define CARTOGRAPHER_BACKEND_TRAJECTORY_BACKEND_2D_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/foundation/sampling.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/mapping/submap_2d.h"
#include "cartographer/backend/constraint_engine_2d.h"
#include "cartographer/backend/pose_optimizer_2d.h"
#include "cartographer/backend/backend_state.h"
#include "cartographer/backend/backend_types.h"
#include "cartographer/backend/backend_trimmer.h"
#include "cartographer/mapping/grid_2d.h"
#include "cartographer/serialization/swmap.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/foundation/sensor_data.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/foundation/transform.h"

namespace cartographer {
namespace mapping {

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
//
// It is extended for submapping:
// Each node has been matched against one or more submaps (adding a constraint
// for each match), both poses of nodes and of submaps are to be optimized.
// All constraints are between a submap i and a node j.
class TrajectoryBackend2D {
 public:
  using InitialTrajectoryPose = mapping::InitialTrajectoryPoseState;

  TrajectoryBackend2D(
      const PoseGraphOptions& options,
      std::unique_ptr<optimization::PoseOptimizer2D> optimization_problem);
  ~TrajectoryBackend2D();

  TrajectoryBackend2D(const TrajectoryBackend2D&) = delete;
  TrajectoryBackend2D& operator=(const TrajectoryBackend2D&) = delete;

  // Adds a new node with 'constant_data'. Its 'constant_data->local_pose' was
  // determined by scan matching against 'insertion_submaps.front()' and the
  // node data was inserted into the 'insertion_submaps'. If
  // 'insertion_submaps.front().finished()' is 'true', data was inserted into
  // this submap for the last time.
  NodeId AddNode(
      std::shared_ptr<const TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
      LOCKS_EXCLUDED(mutex_);

  void AddOdometryData(int trajectory_id,
                       const sensor::OdometryData& odometry_data)
      LOCKS_EXCLUDED(mutex_);

  void DeleteTrajectory(int trajectory_id);
  void FinishTrajectory(int trajectory_id);
  bool IsTrajectoryFinished(int trajectory_id) const
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void FreezeTrajectory(int trajectory_id);
  bool IsTrajectoryFrozen(int trajectory_id) const
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void AddSerializedSubmap(const io::SerializedSubmap2D& submap);
  void AddSerializedNode(const io::SerializedNode& node);
  void AddNodeToSubmap(const NodeId& node_id,
                       const SubmapId& submap_id);
  void AddSerializedConstraints(
      const std::vector<Constraint>& constraints);
  void AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer);
  void RunFinalOptimization();
  SubmapData GetSubmapData(const SubmapId& submap_id) const
      LOCKS_EXCLUDED(mutex_);
  MapById<SubmapId, SubmapData> GetAllSubmapData() const
      LOCKS_EXCLUDED(mutex_);
  // Full-map branch-and-bound for a robot that was lifted or tilted.
  // kRejected: the scan does not lock anywhere.
  // kStay: the best pose is already within 0.8 m of 'current_local'. The
  // chassis was put back near where it was held; mapping may continue there.
  // kJump: a distinct pose far from the held one, clear of the next hypothesis.
  struct Relocalization {
    enum class Decision { kRejected, kStay, kJump };
    Decision decision = Decision::kRejected;
    transform::Rigid2d pose{};
  };
  Relocalization Relocalize(
      const sensor::PointCloud& cloud,
      const transform::Rigid2d& current_local) const
      LOCKS_EXCLUDED(mutex_);
  MapById<SubmapId, SubmapPose> GetAllSubmapPoses() const
      LOCKS_EXCLUDED(mutex_);
  transform::Rigid2d GetLocalToGlobalTransform(int trajectory_id) const
      LOCKS_EXCLUDED(mutex_);
  MapById<NodeId, TrajectoryNode> GetTrajectoryNodes() const
      LOCKS_EXCLUDED(mutex_);
  MapById<NodeId, TrajectoryNodePose> GetTrajectoryNodePoses() const
      LOCKS_EXCLUDED(mutex_);
  std::map<int, TrajectoryState> GetTrajectoryStates() const
      LOCKS_EXCLUDED(mutex_);
  sensor::MapByTime<sensor::OdometryData> GetOdometryData() const
      LOCKS_EXCLUDED(mutex_);
  std::vector<Constraint> constraints() const LOCKS_EXCLUDED(mutex_);
  // Solve timings recorded by the active pose optimizer (ceres|isam2).
  std::vector<optimization::SolveMetric> GetOptimizationSolveMetrics() const
      LOCKS_EXCLUDED(mutex_);
  std::string GetPoseOptimizerBackendName() const LOCKS_EXCLUDED(mutex_);
  void SetInitialTrajectoryPose(int from_trajectory_id, int to_trajectory_id,
                                const transform::Rigid2d& pose,
                                const common::Time time)
      LOCKS_EXCLUDED(mutex_);
  void SetGlobalSlamOptimizationCallback(
      GlobalSlamOptimizationCallback callback);
  transform::Rigid2d GetInterpolatedGlobalTrajectoryPose(
      int trajectory_id, const common::Time time) const
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);

 private:
  struct WorkItem {
    enum class Result {
      kDoNotRunOptimization,
      kRunOptimization,
    };

    std::function<Result()> task;
  };

  MapById<SubmapId, SubmapData> GetSubmapDataUnderLock()
      const EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Handles a new work item.
  void AddWorkItem(const std::function<WorkItem::Result()>& work_item)
      LOCKS_EXCLUDED(mutex_) LOCKS_EXCLUDED(work_queue_mutex_);

  // Adds connectivity and sampler for a trajectory if it does not exist.
  void AddTrajectoryIfNeeded(int trajectory_id)
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Appends the new node and submap (if needed) to the internal data
  // structures.
  NodeId AppendNode(
      std::shared_ptr<const TrajectoryNode::Data> constant_data,
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps,
      const transform::Rigid2d& optimized_pose) LOCKS_EXCLUDED(mutex_);

  // Grows the optimization problem to have an entry for every element of
  // 'insertion_submaps'. Returns the IDs for the 'insertion_submaps'.
  std::vector<SubmapId> InitializeGlobalSubmapPoses(
      int trajectory_id,
      const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps)
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Adds constraints for a node on the serial backend worker.
  WorkItem::Result ComputeConstraintsForNode(
      const NodeId& node_id,
      std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
      bool newly_finished_submap) LOCKS_EXCLUDED(mutex_);

  // Computes constraints for a node and submap pair.
  void ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id)
      LOCKS_EXCLUDED(mutex_);

  // Deletes trajectories waiting for deletion. Must not be called during
  // constraint search.
  void DeleteTrajectoriesIfNeeded() EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Runs the optimization and executes the trimmers.
  void HandleOptimizationResult(
      const constraints::ConstraintEngine2D::Result& result)
      LOCKS_EXCLUDED(mutex_) LOCKS_EXCLUDED(work_queue_mutex_);

  // Serially processes backend work without blocking the frontend thread.
  void BackendWorkerLoop() LOCKS_EXCLUDED(mutex_)
      LOCKS_EXCLUDED(work_queue_mutex_);

  // Inserts a FIFO fence and waits until all earlier backend work has finished.
  void WaitForAllComputations() LOCKS_EXCLUDED(mutex_)
      LOCKS_EXCLUDED(work_queue_mutex_);

  // Runs the optimization. Callers have to make sure, that there is only one
  // optimization being run at a time.
  void RunOptimization() LOCKS_EXCLUDED(mutex_);

  bool CanAddWorkItemModifying(int trajectory_id)
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  // Computes the local to global map frame transform based on the given
  // 'global_submap_poses'.
  transform::Rigid2d ComputeLocalToGlobalTransform(
      const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
      int trajectory_id) const EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  SubmapData GetSubmapDataUnderLock(const SubmapId& submap_id) const
      EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  const PoseGraphOptions options_;
  GlobalSlamOptimizationCallback global_slam_optimization_callback_;
  mutable absl::Mutex mutex_;
  absl::Mutex work_queue_mutex_;

  std::deque<WorkItem> work_queue_ GUARDED_BY(work_queue_mutex_);
  bool worker_running_ GUARDED_BY(work_queue_mutex_) = true;
  std::thread backend_worker_;

  // We globally localize a fraction of the nodes from each trajectory.
  absl::flat_hash_map<int, std::unique_ptr<common::FixedRatioSampler>>
      global_localization_samplers_ GUARDED_BY(mutex_);

  // Number of nodes added since last loop closure.
  int num_nodes_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

  // Current optimization problem.
  std::unique_ptr<optimization::PoseOptimizer2D> optimization_problem_;
  constraints::ConstraintEngine2D constraint_builder_;

  // List of all trimmers to consult when optimizations finish.
  std::vector<std::unique_ptr<PoseGraphTrimmer>> trimmers_ GUARDED_BY(mutex_);

  PoseGraphData data_ GUARDED_BY(mutex_);

  ValueConversionTables conversion_tables_;

  // Allows querying and manipulating the pose graph by the 'trimmers_'. The
  // 'mutex_' of the pose graph is held while this class is used.
  class TrimmingHandle : public Trimmable {
   public:
    TrimmingHandle(TrajectoryBackend2D* parent);
    ~TrimmingHandle() {}

    int num_submaps(int trajectory_id) const;
    std::vector<SubmapId> GetSubmapIds(int trajectory_id) const;
    MapById<SubmapId, SubmapData> GetOptimizedSubmapData() const
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);
    const MapById<NodeId, TrajectoryNode>& GetTrajectoryNodes() const
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);
    const std::vector<Constraint>& GetConstraints() const
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);
    void TrimSubmap(const SubmapId& submap_id)
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);
    bool IsFinished(int trajectory_id) const
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);
    void SetTrajectoryState(int trajectory_id, TrajectoryState state)
        EXCLUSIVE_LOCKS_REQUIRED(parent_->mutex_);

   private:
    TrajectoryBackend2D* const parent_;
  };
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_BACKEND_TRAJECTORY_BACKEND_2D_H_
