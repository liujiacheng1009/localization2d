// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/frontend/frontend_2d.h"

#include <cmath>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer::mapping {
namespace {

}  // namespace

Frontend2D::Frontend2D(
    const TrajectoryBuilderOptions& options,
    sensor::DataDispatcher* const data_dispatcher, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids, std::string range_sensor_id,
    TrajectoryBackend2D* const backend,
    LocalSlamResultCallback local_slam_result_callback,
    const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter)
    : data_dispatcher_(data_dispatcher),
      trajectory_id_(trajectory_id),
      backend_(backend),
      local_slam_(absl::make_unique<LocalSlam2D>(
          options.trajectory_builder_2d_options(),
          std::move(range_sensor_id))),
      local_slam_result_callback_(std::move(local_slam_result_callback)),
      pose_graph_odometry_motion_filter_(pose_graph_odometry_motion_filter) {
  absl::flat_hash_set<std::string> sensor_ids;
  for (const SensorId& sensor_id : expected_sensor_ids) {
    sensor_ids.insert(sensor_id.id);
  }
  data_dispatcher_->AddTrajectory(
      trajectory_id_, sensor_ids,
      [this](const std::string& sensor_id, sensor::SensorData data) {
        HandleSensorData(sensor_id, std::move(data));
      });
}

void Frontend2D::AddSensorData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& data) {
  data_dispatcher_->AddSensorData(trajectory_id_, sensor_id, data);
}

void Frontend2D::AddSensorData(
    const std::string& sensor_id, const sensor::OdometryData& data) {
  data_dispatcher_->AddSensorData(trajectory_id_, sensor_id, data);
}

void Frontend2D::HandleSensorData(
    const std::string& sensor_id, sensor::SensorData data) {
  std::visit(
      [this, &sensor_id](const auto& sensor_data) {
        ProcessSensorData(sensor_id, sensor_data);
      },
      data);
}

void Frontend2D::ProcessSensorData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& data) {
  auto matching_result = local_slam_->AddRangeData(sensor_id, data);
  if (matching_result == nullptr) return;

  transform::Rigid2d published_pose = matching_result->local_pose;
  std::unique_ptr<InsertionResult> insertion_result;
  if (matching_result->tracking_lost) {
    ++lost_holds_;
    const bool due =
        lost_holds_ >= 3 &&
        (last_relocalize_attempt_ == common::Time::min() ||
         common::ToSeconds(matching_result->time - last_relocalize_attempt_) >
             0.5);
    if (lost_holds_ == 1 || lost_holds_ % 10 == 0) {
      LOG(INFO) << "Holding pose after a lift or tilt. holds=" << lost_holds_;
    }
    if (due) {
      last_relocalize_attempt_ = matching_result->time;
      const auto recovered = backend_->Relocalize(
          matching_result->lost_cloud, matching_result->local_pose);
      using Decision = TrajectoryBackend2D::Relocalization::Decision;
      if (recovered.decision == Decision::kRejected) {
        have_relocalize_candidate_ = false;
      } else {
        const bool agrees =
            have_relocalize_candidate_ &&
            candidate_decision_ == recovered.decision &&
            (recovered.pose.translation() - relocalize_candidate_.translation())
                    .norm() < 0.40 &&
            std::abs(transform::Yaw(relocalize_candidate_.inverse() *
                                     recovered.pose)) < 0.35;
        if (agrees) {
          const auto local_insertion =
              recovered.decision == Decision::kJump
                  ? local_slam_->CommitRelocalizedPose(
                        matching_result->time, recovered.pose,
                        matching_result->tracking_range_data)
                  : local_slam_->ResumeMapping(
                        matching_result->time, recovered.pose,
                        matching_result->tracking_range_data);
          published_pose = recovered.pose;
          lost_holds_ = 0;
          have_relocalize_candidate_ = false;
          LOG(INFO) << "Relocalization confirmed at "
                    << recovered.pose.translation().x() << ", "
                    << recovered.pose.translation().y();
          if (local_insertion != nullptr) {
            const NodeId node_id = backend_->AddNode(
                local_insertion->constant_data, trajectory_id_,
                local_insertion->insertion_submaps);
            CHECK_EQ(node_id.trajectory_id, trajectory_id_);
            insertion_result = absl::make_unique<InsertionResult>(InsertionResult{
                node_id, local_insertion->constant_data,
                std::vector<std::shared_ptr<const Submap>>(
                    local_insertion->insertion_submaps.begin(),
                    local_insertion->insertion_submaps.end())});
          }
        } else {
          relocalize_candidate_ = recovered.pose;
          candidate_decision_ = recovered.decision;
          have_relocalize_candidate_ = true;
          LOG(INFO) << "Relocalization candidate at "
                    << recovered.pose.translation().x() << ", "
                    << recovered.pose.translation().y()
                    << ", waiting for a second scan";
        }
      }
    }
  } else {
    lost_holds_ = 0;
    have_relocalize_candidate_ = false;
    if (matching_result->insertion_result != nullptr) {
      const NodeId node_id = backend_->AddNode(
          matching_result->insertion_result->constant_data, trajectory_id_,
          matching_result->insertion_result->insertion_submaps);
      CHECK_EQ(node_id.trajectory_id, trajectory_id_);
      insertion_result = absl::make_unique<InsertionResult>(InsertionResult{
          node_id, matching_result->insertion_result->constant_data,
          std::vector<std::shared_ptr<const Submap>>(
              matching_result->insertion_result->insertion_submaps.begin(),
              matching_result->insertion_result->insertion_submaps.end())});
    }
  }
  if (local_slam_result_callback_) {
    local_slam_result_callback_(
        trajectory_id_, matching_result->time, published_pose,
        std::move(matching_result->range_data_in_local),
        std::move(insertion_result));
  }
}

void Frontend2D::SetChassisTilt(const double tilt_radians) {
  local_slam_->SetChassisTilt(tilt_radians);
}

void Frontend2D::ProcessSensorData(
    const std::string&, const sensor::OdometryData& data) {
  CHECK(data.pose.translation().allFinite());
  CHECK(std::isfinite(transform::Yaw(data.pose)));
  local_slam_->AddOdometryData(data);
  if (pose_graph_odometry_motion_filter_.has_value() &&
      pose_graph_odometry_motion_filter_->IsSimilar(data.time, data.pose)) {
    return;
  }
  backend_->AddOdometryData(trajectory_id_, data);
}

}  // namespace cartographer::mapping
