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

#include "cartographer/backend/trajectory_backend_2d.h"
#include "cartographer/mapping/grid_2d.h"

#ifndef WIN32
#include <unistd.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/backend/overlapping_submaps_trimmer_2d.h"
#include "cartographer/application/slam_options.h"
#include "cartographer/foundation/voxel_filter.h"
#include "cartographer/foundation/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

TrajectoryBackend2D::TrajectoryBackend2D(
    const PoseGraphOptions& options,
    std::unique_ptr<optimization::PoseOptimizer2D> optimization_problem)
    : options_(options),
      optimization_problem_(std::move(optimization_problem)),
      constraint_builder_(options_.constraint_builder_options()) {
  if (options.has_overlapping_submaps_trimmer_2d()) {
    const auto& trimmer_options = options.overlapping_submaps_trimmer_2d();
    AddTrimmer(absl::make_unique<OverlappingSubmapsTrimmer2D>(
        trimmer_options.fresh_submaps_count(),
        trimmer_options.min_covered_area(),
        trimmer_options.min_added_submaps_count()));
  }
  backend_worker_ = std::thread([this] { BackendWorkerLoop(); });
}

TrajectoryBackend2D::~TrajectoryBackend2D() {
  WaitForAllComputations();
  {
    absl::MutexLock locker(&work_queue_mutex_);
    CHECK(worker_running_);
    worker_running_ = false;
  }
  backend_worker_.join();
}

std::vector<SubmapId> TrajectoryBackend2D::InitializeGlobalSubmapPoses(
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {
  CHECK(!insertion_submaps.empty());
  const auto& submap_data = optimization_problem_->submap_data();
  if (insertion_submaps.size() == 1) {
    // If we don't already have an entry for the first submap, add one.
    if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
      optimization_problem_->AddSubmap(
          trajectory_id,
          ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                        trajectory_id) *
              insertion_submaps[0]->local_pose());
    }
    if (submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 1) {
      const SubmapId submap_id{trajectory_id, 0};
      CHECK(data_.submap_data.at(submap_id).submap ==
            insertion_submaps.front());
      return {submap_id};
    }
    // Relocalization replaced the active pair with one new submap. AppendNode
    // has already stored it; the optimization problem still ends at the
    // previous submap.
    const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
    CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);
    const SubmapId last_submap_id = std::prev(end_it)->id;
    if (data_.submap_data.at(last_submap_id).submap ==
        insertion_submaps.front()) {
      return {last_submap_id};
    }
    optimization_problem_->AddSubmap(
        trajectory_id,
        ComputeLocalToGlobalTransform(data_.global_submap_poses_2d,
                                      trajectory_id) *
            insertion_submaps.front()->local_pose());
    return {SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }
  CHECK_EQ(2, insertion_submaps.size());
  const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
  CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);
  const SubmapId last_submap_id = std::prev(end_it)->id;
  if (data_.submap_data.at(last_submap_id).submap ==
      insertion_submaps.front()) {
    // In this case, 'last_submap_id' is the ID of
    // 'insertions_submaps.front()' and 'insertions_submaps.back()' is new.
    const auto& first_submap_pose = submap_data.at(last_submap_id).global_pose;
    optimization_problem_->AddSubmap(
        trajectory_id,
        first_submap_pose *
            insertion_submaps[0]->local_pose().inverse() *
            insertion_submaps[1]->local_pose());
    return {last_submap_id,
            SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
  }
  CHECK(data_.submap_data.at(last_submap_id).submap ==
        insertion_submaps.back());
  const SubmapId front_submap_id{trajectory_id,
                                 last_submap_id.submap_index - 1};
  CHECK(data_.submap_data.at(front_submap_id).submap ==
        insertion_submaps.front());
  return {front_submap_id, last_submap_id};
}

NodeId TrajectoryBackend2D::AppendNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps,
    const transform::Rigid2d& optimized_pose) {
  absl::MutexLock locker(&mutex_);
  AddTrajectoryIfNeeded(trajectory_id);
  if (!CanAddWorkItemModifying(trajectory_id)) {
    LOG(WARNING) << "AddNode was called for finished or deleted trajectory.";
  }
  const NodeId node_id = data_.trajectory_nodes.Append(
      trajectory_id, TrajectoryNode{constant_data, optimized_pose});
  ++data_.num_trajectory_nodes;
  // Test if the 'insertion_submap.back()' is one we never saw before.
  if (data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
      std::prev(data_.submap_data.EndOfTrajectory(trajectory_id))
              ->data.submap != insertion_submaps.back()) {
    // We grow 'data_.submap_data' as needed. This code assumes that the first
    // time we see a new submap is as 'insertion_submaps.back()'.
    const SubmapId submap_id =
        data_.submap_data.Append(trajectory_id, InternalSubmapData());
    data_.submap_data.at(submap_id).submap = insertion_submaps.back();
    LOG(INFO) << "Inserted submap " << submap_id << ".";
  }
  return node_id;
}

NodeId TrajectoryBackend2D::AddNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {
  const transform::Rigid2d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);

  const NodeId node_id = AppendNode(constant_data, trajectory_id,
                                    insertion_submaps, optimized_pose);
  // We have to check this here, because it might have changed by the time we
  // execute the lambda.
  const bool newly_finished_submap =
      insertion_submaps.front()->insertion_finished();
  AddWorkItem([=, this]() LOCKS_EXCLUDED(mutex_) {
    return ComputeConstraintsForNode(node_id, insertion_submaps,
                                     newly_finished_submap);
  });
  return node_id;
}

void TrajectoryBackend2D::AddWorkItem(
    const std::function<WorkItem::Result()>& work_item) {
  absl::MutexLock locker(&work_queue_mutex_);
  CHECK(worker_running_);
  work_queue_.push_back({work_item});
}

void TrajectoryBackend2D::AddTrajectoryIfNeeded(const int trajectory_id) {
  data_.trajectories_state[trajectory_id];
  CHECK(data_.trajectories_state.at(trajectory_id).state !=
        TrajectoryState::FINISHED);
  CHECK(data_.trajectories_state.at(trajectory_id).state !=
        TrajectoryState::DELETED);
  CHECK(data_.trajectories_state.at(trajectory_id).deletion_state ==
        InternalTrajectoryState::DeletionState::NORMAL);
  // Make sure we have a sampler for this trajectory.
  if (!global_localization_samplers_[trajectory_id]) {
    global_localization_samplers_[trajectory_id] =
        absl::make_unique<common::FixedRatioSampler>(
            options_.global_sampling_ratio());
  }
}

void TrajectoryBackend2D::AddOdometryData(const int trajectory_id,
                                  const sensor::OdometryData& odometry_data) {
  AddWorkItem([=, this]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(trajectory_id)) {
      optimization_problem_->AddOdometryData(trajectory_id, odometry_data);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void TrajectoryBackend2D::ComputeConstraint(const NodeId& node_id,
                                    const SubmapId& submap_id) {
  bool maybe_add_local_constraint = false;
  bool maybe_add_global_constraint = false;
  const TrajectoryNode::Data* constant_data;
  const Submap2D* submap;
  {
    absl::MutexLock locker(&mutex_);
    CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);
    if (!data_.submap_data.at(submap_id).submap->insertion_finished()) {
      // Uplink server only receives grids when they are finished, so skip
      // constraint search before that.
      return;
    }

    if (node_id.trajectory_id == submap_id.trajectory_id) {
      // Same-trajectory constraints can be matched in a local search window.
      maybe_add_local_constraint = true;
    } else if (global_localization_samplers_[node_id.trajectory_id]->Pulse()) {
      maybe_add_global_constraint = true;
    }
    constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
    submap = static_cast<const Submap2D*>(
        data_.submap_data.at(submap_id).submap.get());
  }

  if (maybe_add_local_constraint) {
    const transform::Rigid2d initial_relative_pose =
        optimization_problem_->submap_data()
            .at(submap_id)
            .global_pose.inverse() *
        optimization_problem_->node_data().at(node_id).global_pose_2d;
    constraint_builder_.MaybeAddConstraint(
        submap_id, submap, node_id, constant_data, initial_relative_pose);
  } else if (maybe_add_global_constraint) {
    constraint_builder_.MaybeAddGlobalConstraint(submap_id, submap, node_id,
                                                 constant_data);
  }
}

TrajectoryBackend2D::WorkItem::Result
TrajectoryBackend2D::ComputeConstraintsForNode(
    const NodeId& node_id,
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps,
    const bool newly_finished_submap) {
  std::vector<SubmapId> submap_ids;
  std::vector<SubmapId> finished_submap_ids;
  std::set<NodeId> newly_finished_submap_node_ids;
  {
    absl::MutexLock locker(&mutex_);
    const auto& constant_data =
        data_.trajectory_nodes.at(node_id).constant_data;
    submap_ids = InitializeGlobalSubmapPoses(node_id.trajectory_id,
                                             insertion_submaps);
    CHECK_EQ(submap_ids.size(), insertion_submaps.size());
    const SubmapId matching_id = submap_ids.front();
    const transform::Rigid2d local_pose_2d = constant_data->local_pose;
    const transform::Rigid2d global_pose_2d =
        optimization_problem_->submap_data().at(matching_id).global_pose *
        insertion_submaps.front()->local_pose().inverse() *
        local_pose_2d;
    optimization_problem_->AddTrajectoryNode(
        matching_id.trajectory_id,
        optimization::NodeSpec2D{constant_data->time, local_pose_2d,
                                 global_pose_2d});
    for (size_t i = 0; i < insertion_submaps.size(); ++i) {
      const SubmapId submap_id = submap_ids[i];
      // Even if this was the last node added to 'submap_id', the submap will
      // only be marked as finished in 'data_.submap_data' further below.
      CHECK(data_.submap_data.at(submap_id).state ==
            SubmapState::kNoConstraintSearch);
      data_.submap_data.at(submap_id).node_ids.emplace(node_id);
      const transform::Rigid2d constraint_transform =
          insertion_submaps[i]->local_pose().inverse() *
          local_pose_2d;
      data_.constraints.push_back(
          Constraint{submap_id,
                     node_id,
                     {constraint_transform,
                      options_.matcher_translation_weight(),
                      options_.matcher_rotation_weight()},
                     Constraint::INTRA_SUBMAP});
    }

    // TODO(gaschler): Consider not searching for constraints against
    // trajectories scheduled for deletion.
    // TODO(danielsievers): Add a member variable and avoid having to copy
    // them out here.
    for (const auto& submap_id_data : data_.submap_data) {
      if (submap_id_data.data.state == SubmapState::kFinished) {
        CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);
        finished_submap_ids.emplace_back(submap_id_data.id);
      }
    }
    if (newly_finished_submap) {
      const SubmapId newly_finished_submap_id = submap_ids.front();
      InternalSubmapData& finished_submap_data =
          data_.submap_data.at(newly_finished_submap_id);
      CHECK(finished_submap_data.state == SubmapState::kNoConstraintSearch);
      finished_submap_data.state = SubmapState::kFinished;
      newly_finished_submap_node_ids = finished_submap_data.node_ids;
    }
  }

  for (const auto& submap_id : finished_submap_ids) {
    ComputeConstraint(node_id, submap_id);
  }

  if (newly_finished_submap) {
    const SubmapId newly_finished_submap_id = submap_ids.front();
    // We have a new completed submap, so we look into adding constraints for
    // old nodes.
    for (const auto& node_id_data : optimization_problem_->node_data()) {
      const NodeId& node_id = node_id_data.id;
      if (newly_finished_submap_node_ids.count(node_id) == 0) {
        ComputeConstraint(node_id, newly_finished_submap_id);
      }
    }
  }
  absl::MutexLock locker(&mutex_);
  ++num_nodes_since_last_loop_closure_;
  if (options_.optimize_every_n_nodes() > 0 &&
      num_nodes_since_last_loop_closure_ > options_.optimize_every_n_nodes()) {
    return WorkItem::Result::kRunOptimization;
  }
  return WorkItem::Result::kDoNotRunOptimization;
}

void TrajectoryBackend2D::DeleteTrajectoriesIfNeeded() {
  TrimmingHandle trimming_handle(this);
  for (auto& it : data_.trajectories_state) {
    if (it.second.deletion_state ==
        InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION) {
      // TODO(gaschler): Consider directly deleting from data_, which may be
      // more complete.
      auto submap_ids = trimming_handle.GetSubmapIds(it.first);
      for (auto& submap_id : submap_ids) {
        trimming_handle.TrimSubmap(submap_id);
      }
      it.second.state = TrajectoryState::DELETED;
      it.second.deletion_state = InternalTrajectoryState::DeletionState::NORMAL;
    }
  }
}

void TrajectoryBackend2D::HandleOptimizationResult(
    const constraints::ConstraintEngine2D::Result& result) {
  {
    absl::MutexLock locker(&mutex_);
    data_.constraints.insert(data_.constraints.end(), result.begin(),
                             result.end());
  }
  RunOptimization();

  if (global_slam_optimization_callback_) {
    std::map<int, NodeId> trajectory_id_to_last_optimized_node_id;
    std::map<int, SubmapId> trajectory_id_to_last_optimized_submap_id;
    {
      absl::MutexLock locker(&mutex_);
      const auto& submap_data = optimization_problem_->submap_data();
      const auto& node_data = optimization_problem_->node_data();
      for (const int trajectory_id : node_data.trajectory_ids()) {
        if (node_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
            submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0) {
          continue;
        }
        trajectory_id_to_last_optimized_node_id.emplace(
            trajectory_id,
            std::prev(node_data.EndOfTrajectory(trajectory_id))->id);
        trajectory_id_to_last_optimized_submap_id.emplace(
            trajectory_id,
            std::prev(submap_data.EndOfTrajectory(trajectory_id))->id);
      }
    }
    global_slam_optimization_callback_(
        trajectory_id_to_last_optimized_submap_id,
        trajectory_id_to_last_optimized_node_id);
  }

  {
    absl::MutexLock locker(&mutex_);
    DeleteTrajectoriesIfNeeded();
    TrimmingHandle trimming_handle(this);
    for (auto& trimmer : trimmers_) {
      trimmer->Trim(&trimming_handle);
    }
    std::erase_if(trimmers_, [](const auto& trimmer) {
      return trimmer->IsFinished();
    });

    num_nodes_since_last_loop_closure_ = 0;
  }
}

void TrajectoryBackend2D::BackendWorkerLoop() {
#ifdef __linux__
  CHECK_NE(nice(10), -1);
#endif
  const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(work_queue_mutex_) {
    return !work_queue_.empty() || !worker_running_;
  };
  for (;;) {
    WorkItem work_item;
    {
      absl::MutexLock locker(&work_queue_mutex_);
      work_queue_mutex_.Await(absl::Condition(&predicate));
      if (work_queue_.empty()) {
        CHECK(!worker_running_);
        return;
      }
      work_item = std::move(work_queue_.front());
      work_queue_.pop_front();
    }
    if (work_item.task() == WorkItem::Result::kRunOptimization) {
      HandleOptimizationResult(constraint_builder_.TakeConstraints());
    }
  }
}

void TrajectoryBackend2D::WaitForAllComputations() {
  std::promise<void> done;
  auto future = done.get_future();
  AddWorkItem([this, &done] {
    auto result = constraint_builder_.TakeConstraints();
    {
      absl::MutexLock locker(&mutex_);
      data_.constraints.insert(data_.constraints.end(), result.begin(),
                               result.end());
    }
    done.set_value();
    return WorkItem::Result::kDoNotRunOptimization;
  });
  future.wait();
}

void TrajectoryBackend2D::DeleteTrajectory(const int trajectory_id) {
  {
    absl::MutexLock locker(&mutex_);
    auto it = data_.trajectories_state.find(trajectory_id);
    if (it == data_.trajectories_state.end()) {
      LOG(WARNING) << "Skipping request to delete non-existing trajectory_id: "
                   << trajectory_id;
      return;
    }
    it->second.deletion_state =
        InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION;
  }
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    CHECK(data_.trajectories_state.at(trajectory_id).state !=
          TrajectoryState::ACTIVE);
    CHECK(data_.trajectories_state.at(trajectory_id).state !=
          TrajectoryState::DELETED);
    CHECK(data_.trajectories_state.at(trajectory_id).deletion_state ==
          InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION);
    data_.trajectories_state.at(trajectory_id).deletion_state =
        InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION;
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void TrajectoryBackend2D::FinishTrajectory(const int trajectory_id) {
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    CHECK(!IsTrajectoryFinished(trajectory_id));
    data_.trajectories_state[trajectory_id].state = TrajectoryState::FINISHED;

    for (const auto& submap : data_.submap_data.trajectory(trajectory_id)) {
      data_.submap_data.at(submap.id).state = SubmapState::kFinished;
    }
    return WorkItem::Result::kRunOptimization;
  });
}

bool TrajectoryBackend2D::IsTrajectoryFinished(const int trajectory_id) const {
  return data_.trajectories_state.count(trajectory_id) != 0 &&
         data_.trajectories_state.at(trajectory_id).state ==
             TrajectoryState::FINISHED;
}

void TrajectoryBackend2D::FreezeTrajectory(const int trajectory_id) {
  AddWorkItem([this, trajectory_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    CHECK(!IsTrajectoryFrozen(trajectory_id));
    data_.trajectories_state[trajectory_id].state = TrajectoryState::FROZEN;
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

bool TrajectoryBackend2D::IsTrajectoryFrozen(const int trajectory_id) const {
  return data_.trajectories_state.count(trajectory_id) != 0 &&
         data_.trajectories_state.at(trajectory_id).state ==
             TrajectoryState::FROZEN;
}

void TrajectoryBackend2D::AddSerializedSubmap(const io::SerializedSubmap2D& value) {
  const transform::Rigid2d global_pose = value.global_pose;
  auto grid = absl::make_unique<ProbabilityGrid>(
      MapLimits(value.grid.resolution, value.grid.max, value.grid.cell_limits),
      value.grid.cells, value.grid.known_cells_box,
      value.grid.min_correspondence_cost, value.grid.max_correspondence_cost,
      &conversion_tables_);
  const std::shared_ptr<const Submap2D> submap =
      std::make_shared<const Submap2D>(value.local_pose, value.num_range_data,
                                       value.finished, std::move(grid),
                                       &conversion_tables_);
  {
    absl::MutexLock locker(&mutex_);
    AddTrajectoryIfNeeded(value.id.trajectory_id);
    if (!CanAddWorkItemModifying(value.id.trajectory_id)) return;
    data_.submap_data.Insert(value.id, InternalSubmapData());
    data_.submap_data.at(value.id).submap = submap;
    data_.global_submap_poses_2d.Insert(
        value.id, optimization::SubmapSpec2D{global_pose});
  }
  AddWorkItem([this, id = value.id, global_pose]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    data_.submap_data.at(id).state = SubmapState::kFinished;
    optimization_problem_->InsertSubmap(id, global_pose);
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void TrajectoryBackend2D::AddSerializedNode(const io::SerializedNode& value) {
  auto constant_data = std::make_shared<const TrajectoryNode::Data>(value.data);
  {
    absl::MutexLock locker(&mutex_);
    AddTrajectoryIfNeeded(value.id.trajectory_id);
    if (!CanAddWorkItemModifying(value.id.trajectory_id)) return;
    data_.trajectory_nodes.Insert(
        value.id, TrajectoryNode{constant_data, value.global_pose});
  }
  AddWorkItem([this, id = value.id, global_pose = value.global_pose]()
                  LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    const auto& data = data_.trajectory_nodes.at(id).constant_data;
    optimization_problem_->InsertTrajectoryNode(
        id, optimization::NodeSpec2D{data->time, data->local_pose,
                                     global_pose});
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void TrajectoryBackend2D::AddNodeToSubmap(const NodeId& node_id,
                                  const SubmapId& submap_id) {
  AddWorkItem([this, node_id, submap_id]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    if (CanAddWorkItemModifying(submap_id.trajectory_id)) {
      data_.submap_data.at(submap_id).node_ids.insert(node_id);
    }
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void TrajectoryBackend2D::AddSerializedConstraints(
    const std::vector<Constraint>& constraints) {
  AddWorkItem([this, constraints]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    for (const auto& constraint : constraints) {
      CHECK(data_.trajectory_nodes.Contains(constraint.node_id));
      CHECK(data_.submap_data.Contains(constraint.submap_id));
      CHECK(data_.trajectory_nodes.at(constraint.node_id).constant_data !=
            nullptr);
      CHECK(data_.submap_data.at(constraint.submap_id).submap != nullptr);
      switch (constraint.tag) {
        case Constraint::Tag::INTRA_SUBMAP:
          CHECK(data_.submap_data.at(constraint.submap_id)
                    .node_ids.emplace(constraint.node_id)
                    .second);
          break;
        case Constraint::Tag::INTER_SUBMAP:
          break;
      }
      const Constraint::Pose pose = constraint.pose;
      data_.constraints.push_back(Constraint{
          constraint.submap_id, constraint.node_id, pose, constraint.tag});
    }
    LOG(INFO) << "Loaded " << constraints.size() << " constraints.";
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void TrajectoryBackend2D::AddTrimmer(std::unique_ptr<PoseGraphTrimmer> trimmer) {
  // C++11 does not allow us to move a unique_ptr into a lambda.
  PoseGraphTrimmer* const trimmer_ptr = trimmer.release();
  AddWorkItem([this, trimmer_ptr]() LOCKS_EXCLUDED(mutex_) {
    absl::MutexLock locker(&mutex_);
    trimmers_.emplace_back(trimmer_ptr);
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

void TrajectoryBackend2D::RunFinalOptimization() {
  {
    AddWorkItem([this]() LOCKS_EXCLUDED(mutex_) {
      absl::MutexLock locker(&mutex_);
      optimization_problem_->SetMaxNumIterations(
          options_.max_num_final_iterations());
      return WorkItem::Result::kRunOptimization;
    });
    AddWorkItem([this]() LOCKS_EXCLUDED(mutex_) {
      absl::MutexLock locker(&mutex_);
      optimization_problem_->SetMaxNumIterations(
          options_.optimization_problem_options()
              .ceres_solver_options()
              .max_num_iterations());
      return WorkItem::Result::kDoNotRunOptimization;
    });
  }
  WaitForAllComputations();
}

void TrajectoryBackend2D::RunOptimization() {
  if (optimization_problem_->submap_data().empty()) {
    return;
  }

  // No other thread is accessing the optimization_problem_,
  // data_.constraints and data_.frozen_trajectories
  // when executing the Solve. Solve is time consuming, so not taking the mutex
  // before Solve to avoid blocking foreground processing.
  optimization_problem_->Solve(data_.constraints, GetTrajectoryStates());
  absl::MutexLock locker(&mutex_);

  const auto& submap_data = optimization_problem_->submap_data();
  const auto& node_data = optimization_problem_->node_data();
  for (const int trajectory_id : node_data.trajectory_ids()) {
    for (const auto& node : node_data.trajectory(trajectory_id)) {
      auto& mutable_trajectory_node = data_.trajectory_nodes.at(node.id);
      mutable_trajectory_node.global_pose = node.data.global_pose_2d;
    }

    // Extrapolate all point cloud poses that were not included in the
    // 'optimization_problem_' yet.
    const auto local_to_new_global =
        ComputeLocalToGlobalTransform(submap_data, trajectory_id);
    const auto local_to_old_global = ComputeLocalToGlobalTransform(
        data_.global_submap_poses_2d, trajectory_id);
    const transform::Rigid2d old_global_to_new_global =
        local_to_new_global * local_to_old_global.inverse();

    const NodeId last_optimized_node_id =
        std::prev(node_data.EndOfTrajectory(trajectory_id))->id;
    auto node_it =
        std::next(data_.trajectory_nodes.find(last_optimized_node_id));
    for (; node_it != data_.trajectory_nodes.EndOfTrajectory(trajectory_id);
         ++node_it) {
      auto& mutable_trajectory_node = data_.trajectory_nodes.at(node_it->id);
      mutable_trajectory_node.global_pose =
          old_global_to_new_global * mutable_trajectory_node.global_pose;
    }
  }
  data_.global_submap_poses_2d = submap_data;
}

bool TrajectoryBackend2D::CanAddWorkItemModifying(int trajectory_id) {
  auto it = data_.trajectories_state.find(trajectory_id);
  if (it == data_.trajectories_state.end()) {
    return true;
  }
  if (it->second.state == TrajectoryState::FINISHED) {
    // TODO(gaschler): Replace all FATAL to WARNING after some testing.
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has finished "
                  "but modification is requested, skipping.";
    return false;
  }
  if (it->second.deletion_state !=
      InternalTrajectoryState::DeletionState::NORMAL) {
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has been scheduled for deletion "
                  "but modification is requested, skipping.";
    return false;
  }
  if (it->second.state == TrajectoryState::DELETED) {
    LOG(FATAL) << "trajectory_id " << trajectory_id
               << " has been deleted "
                  "but modification is requested, skipping.";
    return false;
  }
  return true;
}

TrajectoryBackend2D::TrimmingHandle::TrimmingHandle(TrajectoryBackend2D* const parent)
    : parent_(parent) {}

int TrajectoryBackend2D::TrimmingHandle::num_submaps(const int trajectory_id) const {
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  return submap_data.SizeOfTrajectoryOrZero(trajectory_id);
}

MapById<SubmapId, SubmapData>
TrajectoryBackend2D::TrimmingHandle::GetOptimizedSubmapData() const {
  MapById<SubmapId, SubmapData> submaps;
  for (const auto& submap_id_data : parent_->data_.submap_data) {
    if (submap_id_data.data.state != SubmapState::kFinished ||
        !parent_->data_.global_submap_poses_2d.Contains(submap_id_data.id)) {
      continue;
    }
    submaps.Insert(
        submap_id_data.id,
        SubmapData{submap_id_data.data.submap,
                   parent_->data_.global_submap_poses_2d
                       .at(submap_id_data.id)
                       .global_pose});
  }
  return submaps;
}

std::vector<SubmapId> TrajectoryBackend2D::TrimmingHandle::GetSubmapIds(
    int trajectory_id) const {
  std::vector<SubmapId> submap_ids;
  const auto& submap_data = parent_->optimization_problem_->submap_data();
  for (const auto& it : submap_data.trajectory(trajectory_id)) {
    submap_ids.push_back(it.id);
  }
  return submap_ids;
}

const MapById<NodeId, TrajectoryNode>&
TrajectoryBackend2D::TrimmingHandle::GetTrajectoryNodes() const {
  return parent_->data_.trajectory_nodes;
}

const std::vector<Constraint>&
TrajectoryBackend2D::TrimmingHandle::GetConstraints() const {
  return parent_->data_.constraints;
}

bool TrajectoryBackend2D::TrimmingHandle::IsFinished(const int trajectory_id) const {
  return parent_->IsTrajectoryFinished(trajectory_id);
}

void TrajectoryBackend2D::TrimmingHandle::SetTrajectoryState(int trajectory_id,
                                                     TrajectoryState state) {
  parent_->data_.trajectories_state[trajectory_id].state = state;
}

void TrajectoryBackend2D::TrimmingHandle::TrimSubmap(const SubmapId& submap_id) {
  // TODO(hrapp): We have to make sure that the trajectory has been finished
  // if we want to delete the last submaps.
  CHECK(parent_->data_.submap_data.at(submap_id).state ==
        SubmapState::kFinished);

  // Compile all nodes that are still INTRA_SUBMAP constrained to other submaps
  // once the submap with 'submap_id' is gone.
  // We need to use node_ids instead of constraints here to be also compatible
  // with frozen trajectories that don't have intra-constraints.
  std::set<NodeId> nodes_to_retain;
  for (const auto& submap_data : parent_->data_.submap_data) {
    if (submap_data.id != submap_id) {
      nodes_to_retain.insert(submap_data.data.node_ids.begin(),
                             submap_data.data.node_ids.end());
    }
  }

  // Remove all nodes that are exlusively associated to 'submap_id'.
  std::set<NodeId> nodes_to_remove;
  std::set_difference(parent_->data_.submap_data.at(submap_id).node_ids.begin(),
                      parent_->data_.submap_data.at(submap_id).node_ids.end(),
                      nodes_to_retain.begin(), nodes_to_retain.end(),
                      std::inserter(nodes_to_remove, nodes_to_remove.begin()));

  // Remove all 'data_.constraints' related to 'submap_id'.
  {
    std::vector<Constraint> constraints;
    for (const Constraint& constraint : parent_->data_.constraints) {
      if (constraint.submap_id != submap_id) {
        constraints.push_back(constraint);
      }
    }
    parent_->data_.constraints = std::move(constraints);
  }

  // Remove all 'data_.constraints' related to 'nodes_to_remove'.
  // If the removal lets other submaps lose all their inter-submap constraints,
  // delete their corresponding constraint submap matchers to save memory.
  {
    std::vector<Constraint> constraints;
    std::set<SubmapId> other_submap_ids_losing_constraints;
    for (const Constraint& constraint : parent_->data_.constraints) {
      if (nodes_to_remove.count(constraint.node_id) == 0) {
        constraints.push_back(constraint);
      } else {
        // A constraint to another submap will be removed, mark it as affected.
        other_submap_ids_losing_constraints.insert(constraint.submap_id);
      }
    }
    parent_->data_.constraints = std::move(constraints);
    // Go through the remaining constraints to ensure we only delete scan
    // matchers of other submaps that have no inter-submap constraints left.
    for (const Constraint& constraint : parent_->data_.constraints) {
      if (constraint.tag == Constraint::Tag::INTRA_SUBMAP) {
        continue;
      } else if (other_submap_ids_losing_constraints.count(
                     constraint.submap_id)) {
        // This submap still has inter-submap constraints - ignore it.
        other_submap_ids_losing_constraints.erase(constraint.submap_id);
      }
    }
    // Delete scan matchers of the submaps that lost all constraints.
    // TODO(wohe): An improvement to this implementation would be to add the
    // caching logic at the constraint builder which could keep around only
    // recently used scan matchers.
    for (const SubmapId& submap_id : other_submap_ids_losing_constraints) {
      parent_->constraint_builder_.DeleteScanMatcher(submap_id);
    }
  }

  // Mark the submap with 'submap_id' as trimmed and remove its data.
  CHECK(parent_->data_.submap_data.at(submap_id).state ==
        SubmapState::kFinished);
  parent_->data_.submap_data.Trim(submap_id);
  parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  parent_->optimization_problem_->TrimSubmap(submap_id);

  // Remove the 'nodes_to_remove' from the pose graph and the optimization
  // problem.
  for (const NodeId& node_id : nodes_to_remove) {
    parent_->data_.trajectory_nodes.Trim(node_id);
    parent_->optimization_problem_->TrimTrajectoryNode(node_id);
  }
}

MapById<SubmapId, SubmapData>
TrajectoryBackend2D::GetSubmapDataUnderLock() const {
  MapById<SubmapId, SubmapData> submaps;
  for (const auto& submap_id_data : data_.submap_data) {
    submaps.Insert(submap_id_data.id,
                   GetSubmapDataUnderLock(submap_id_data.id));
  }
  return submaps;
}

void TrajectoryBackend2D::SetGlobalSlamOptimizationCallback(
    GlobalSlamOptimizationCallback callback) {
  global_slam_optimization_callback_ = callback;
}

}  // namespace mapping
}  // namespace cartographer
