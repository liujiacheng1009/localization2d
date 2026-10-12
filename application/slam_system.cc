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

#include "cartographer/application/slam_system.h"

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/serialization/swmap.h"
#include "cartographer/frontend/local_slam_2d.h"
#include "cartographer/backend/trajectory_backend_2d.h"
#include "cartographer/frontend/frontend_2d.h"
#include "cartographer/frontend/motion_filter.h"
#include "cartographer/frontend/data_dispatcher.h"
#include "cartographer/foundation/voxel_filter.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/foundation/transform.h"

namespace cartographer {
namespace mapping {

SlamSystemOptions CreateSlamSystemOptions(
    common::ParameterDictionary* const parameter_dictionary) {
  SlamSystemOptions options;
  options.set_use_trajectory_builder_2d(
      parameter_dictionary->GetBool("use_trajectory_builder_2d"));
  options.set_collate_by_trajectory(
      parameter_dictionary->GetBool("collate_by_trajectory"));
  *options.mutable_pose_graph_options() = CreatePoseGraphOptions(
      parameter_dictionary->GetDictionary("pose_graph").get());
  CHECK(options.use_trajectory_builder_2d());
  return options;
}

namespace {


std::string SelectRangeSensorId(
    const std::set<SlamSystem::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const SlamSystem::SensorId& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == SlamSystem::SensorId::SensorType::RANGE) {
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  CHECK_EQ(range_sensor_ids.size(), 1u)
      << "The bag application requires exactly one range sensor.";
  return range_sensor_ids.front();
}

void MaybeAddPureLocalizationTrimmer(
    const int trajectory_id,
    const TrajectoryBuilderOptions& trajectory_options,
    TrajectoryBackend2D* backend) {
  if (trajectory_options.pure_localization()) {
    LOG(WARNING)
        << "'TrajectoryBuilderOptions::pure_localization' field is deprecated. "
           "Use 'TrajectoryBuilderOptions::pure_localization_trimmer' instead.";
    backend->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id, 3 /* max_submaps_to_keep */));
    return;
  }
  if (trajectory_options.has_pure_localization_trimmer()) {
    backend->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id,
        trajectory_options.pure_localization_trimmer().max_submaps_to_keep()));
  }
}

}  // namespace

SlamSystem::SlamSystem(const SlamSystemOptions& options) : options_(options) {
  CHECK(options.use_trajectory_builder_2d());
  backend_ = absl::make_unique<TrajectoryBackend2D>(
      options_.pose_graph_options(),
      optimization::CreatePoseOptimizer2D(options_.pose_graph_options()));
  LOG(INFO) << "Pose graph optimizer backend: "
            << options_.pose_graph_options().optimizer();
  CHECK(!options.collate_by_trajectory());
  data_dispatcher_ = absl::make_unique<sensor::DataDispatcher>();
}

int SlamSystem::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  const int trajectory_id = trajectory_builders_.size();

  absl::optional<MotionFilter> pose_graph_odometry_motion_filter;
  if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";
    pose_graph_odometry_motion_filter.emplace(
        MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  }

  trajectory_builders_.push_back(absl::make_unique<Frontend2D>(
      trajectory_options, data_dispatcher_.get(), trajectory_id,
      expected_sensor_ids, SelectRangeSensorId(expected_sensor_ids),
      backend_.get(), std::move(local_slam_result_callback),
      pose_graph_odometry_motion_filter));
  MaybeAddPureLocalizationTrimmer(trajectory_id, trajectory_options,
                                  backend_.get());

  if (trajectory_options.has_initial_trajectory_pose()) {
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    backend_->SetInitialTrajectoryPose(
        trajectory_id, initial_trajectory_pose.to_trajectory_id,
        initial_trajectory_pose.relative_pose, initial_trajectory_pose.timestamp);
  }
  return trajectory_id;
}

int SlamSystem::AddTrajectoryForDeserialization() {
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.emplace_back();
  return trajectory_id;
}

void SlamSystem::FinishTrajectory(const int trajectory_id) {
  data_dispatcher_->FinishTrajectory(trajectory_id);
  backend_->FinishTrajectory(trajectory_id);
}

std::string SlamSystem::GetSubmapTexture(
    const SubmapId& submap_id, SubmapTextureResponse* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  const auto submap_data = backend_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }
  submap_data.submap->ToSubmapTextureResponse(submap_data.pose, response);
  return "";
}

bool SlamSystem::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  return io::WriteSwMap(filename, *backend_, include_unfinished_submaps);
}

std::map<int, int> SlamSystem::LoadStateFromFile(
    const std::string& state_filename, const bool load_frozen_state) {
  const std::string suffix = ".swmap";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a .swmap file.";
  }
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  CHECK(load_frozen_state)
      << "Native .swmap loading currently supports frozen maps only.";
  io::SerializedState state = io::ReadSwMap(state_filename);
  std::map<int, int> trajectory_remapping;
  for (int old_id : state.trajectory_ids) {
    const int new_id = AddTrajectoryForDeserialization();
    CHECK(trajectory_remapping.emplace(old_id, new_id).second);
    backend_->FreezeTrajectory(new_id);
  }
  for (auto& submap : state.submaps) {
    submap.id.trajectory_id = trajectory_remapping.at(submap.id.trajectory_id);
    backend_->AddSerializedSubmap(submap);
  }
  for (auto& node : state.nodes) {
    node.id.trajectory_id = trajectory_remapping.at(node.id.trajectory_id);
    backend_->AddSerializedNode(node);
  }
  for (auto constraint : state.constraints) {
    constraint.submap_id.trajectory_id =
        trajectory_remapping.at(constraint.submap_id.trajectory_id);
    constraint.node_id.trajectory_id =
        trajectory_remapping.at(constraint.node_id.trajectory_id);
    if (constraint.tag == Constraint::INTRA_SUBMAP) {
      backend_->AddNodeToSubmap(constraint.node_id, constraint.submap_id);
    }
  }
  return trajectory_remapping;
}

std::unique_ptr<SlamSystem> CreateSlamSystem(const SlamSystemOptions& options) {
  return absl::make_unique<SlamSystem>(options);
}

}  // namespace mapping
}  // namespace cartographer
