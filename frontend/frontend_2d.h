// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef CARTOGRAPHER_TRAJECTORY_TRAJECTORY_FRONTEND_2D_H_
#define CARTOGRAPHER_TRAJECTORY_TRAJECTORY_FRONTEND_2D_H_

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "absl/types/optional.h"
#include "cartographer/foundation/time.h"
#include "cartographer/frontend/data_dispatcher.h"
#include "cartographer/frontend/local_slam_2d.h"
#include "cartographer/frontend/motion_filter.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/backend/trajectory_backend_2d.h"
#include "cartographer/application/slam_options.h"

namespace cartographer::mapping {

// The single 2D frontend boundary: orders scan/odometry input, runs local
// SLAM, and forwards nodes and odometry to the pose graph backend.
class Frontend2D {
 public:
  struct InsertionResult {
    NodeId node_id;
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    std::vector<std::shared_ptr<const Submap>> insertion_submaps;
  };

  using LocalSlamResultCallback =
      std::function<void(int, common::Time, transform::Rigid2d,
                         sensor::RangeData,
                         std::unique_ptr<const InsertionResult>)>;

  struct SensorId {
    enum class SensorType { RANGE, ODOMETRY };

    SensorType type;
    std::string id;

    auto operator<=>(const SensorId&) const = default;
  };

  Frontend2D(
      const TrajectoryBuilderOptions& options,
      sensor::DataDispatcher* data_dispatcher, int trajectory_id,
      const std::set<SensorId>& expected_sensor_ids, std::string range_sensor_id,
      TrajectoryBackend2D* backend, LocalSlamResultCallback local_slam_result_callback,
      const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter);

  Frontend2D(const Frontend2D&) = delete;
  Frontend2D& operator=(const Frontend2D&) = delete;

  void AddSensorData(const std::string& sensor_id,
                     const sensor::TimedPointCloudData& data);
  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& data);

 private:
  void HandleSensorData(const std::string& sensor_id, sensor::SensorData data);
  void ProcessSensorData(const std::string& sensor_id,
                         const sensor::TimedPointCloudData& data);
  void ProcessSensorData(const std::string& sensor_id,
                         const sensor::OdometryData& data);

  sensor::DataDispatcher* const data_dispatcher_;
  const int trajectory_id_;
  TrajectoryBackend2D* const backend_;
  std::unique_ptr<LocalSlam2D> local_slam_;
  LocalSlamResultCallback local_slam_result_callback_;
  absl::optional<MotionFilter> pose_graph_odometry_motion_filter_;
  int lost_holds_ = 0;
  common::Time last_relocalize_attempt_ = common::Time::min();
  bool have_relocalize_candidate_ = false;
  transform::Rigid2d relocalize_candidate_;
};

}  // namespace cartographer::mapping

#endif  // CARTOGRAPHER_TRAJECTORY_TRAJECTORY_FRONTEND_2D_H_
