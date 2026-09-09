/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#ifndef CARTOGRAPHER_TRAJECTORY_POSE_EXTRAPOLATOR_H_
#define CARTOGRAPHER_TRAJECTORY_POSE_EXTRAPOLATOR_H_

#include <deque>

#include "cartographer/foundation/transform.h"
#include "cartographer/foundation/sensor_data.h"

namespace cartographer {
namespace mapping {

// Keeps recent planar poses to estimate velocity and extrapolates them using
// odometry when available.
class PoseExtrapolator {
 public:
  explicit PoseExtrapolator(common::Duration pose_queue_duration);

  PoseExtrapolator(const PoseExtrapolator&) = delete;
  PoseExtrapolator& operator=(const PoseExtrapolator&) = delete;

  common::Time GetLastPoseTime() const;
  transform::Rigid2d GetLastPose() const;
  common::Time GetLastExtrapolatedTime() const;

  void AddPose(common::Time time, const transform::Rigid2d& pose);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  transform::Rigid2d ExtrapolatePose(common::Time time);

  // Pose of the tracking frame at `time`, expressed in the tracking frame at
  // `reference_time`. Uses odometry interpolation over the sweep when possible,
  // otherwise a constant body-frame twist. Does not depend on the last SLAM
  // pose, so early beams in a LaserScan can still be destorted.
  transform::Rigid2d TrackingMotion(common::Time time,
                                    common::Time reference_time) const;

 private:
  struct TimedPose {
    common::Time time;
    transform::Rigid2d pose;
  };

  void UpdateVelocitiesFromPoses();
  void TrimOdometryData();
  bool InterpolateOdometry(common::Time time, transform::Rigid2d* pose) const;
  void CurrentBodyTwist(Eigen::Vector2d* linear_velocity_in_tracking,
                        double* angular_velocity) const;

  const common::Duration pose_queue_duration_;
  std::deque<TimedPose> timed_pose_queue_;
  Eigen::Vector2d linear_velocity_from_poses_ = Eigen::Vector2d::Zero();
  double angular_velocity_from_poses_ = 0.;

  std::deque<sensor::OdometryData> odometry_data_;
  Eigen::Vector2d linear_velocity_from_odometry_ = Eigen::Vector2d::Zero();
  double angular_velocity_from_odometry_ = 0.;

  TimedPose cached_extrapolated_pose_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRAJECTORY_POSE_EXTRAPOLATOR_H_
