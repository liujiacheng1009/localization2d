/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "cartographer/frontend/pose_extrapolator.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "cartographer/foundation/utils.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr double kOdometryHistorySeconds = 0.5;
constexpr double kMaxOdometryExtrapolateSeconds = 0.08;

transform::Rigid2d IntegrateBodyTwist(const Eigen::Vector2d& velocity_in_tracking,
                                      const double angular_velocity,
                                      const double delta_t) {
  const double theta = angular_velocity * delta_t;
  if (std::abs(theta) < 1e-6) {
    return transform::MakeRigid2(
        Eigen::Vector2d(velocity_in_tracking * delta_t), theta);
  }
  const double x =
      (velocity_in_tracking.x() * std::sin(theta) +
       velocity_in_tracking.y() * (std::cos(theta) - 1.)) /
      angular_velocity;
  const double y =
      (velocity_in_tracking.y() * std::sin(theta) -
       velocity_in_tracking.x() * (std::cos(theta) - 1.)) /
      angular_velocity;
  return transform::MakeRigid2(Eigen::Vector2d(x, y), theta);
}

}  // namespace

PoseExtrapolator::PoseExtrapolator(
    const common::Duration pose_queue_duration)
    : pose_queue_duration_(pose_queue_duration),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid2d()} {}

common::Time PoseExtrapolator::GetLastPoseTime() const {
  return timed_pose_queue_.empty() ? common::Time::min()
                                   : timed_pose_queue_.back().time;
}

transform::Rigid2d PoseExtrapolator::GetLastPose() const {
  CHECK(!timed_pose_queue_.empty());
  return timed_pose_queue_.back().pose;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  return cached_extrapolated_pose_.time;
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid2d& pose) {
  timed_pose_queue_.push_back({time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  TrimOdometryData();
  cached_extrapolated_pose_ = {time, pose};
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (!timed_pose_queue_.empty() &&
      odometry_data.time + common::FromSeconds(kOdometryHistorySeconds) <
          timed_pose_queue_.back().time) {
    return;
  }
  if (!odometry_data_.empty() &&
      odometry_data.time <= odometry_data_.back().time) {
    return;
  }
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) return;

  const auto& older = odometry_data_[odometry_data_.size() - 2];
  const auto& newer = odometry_data_.back();
  const double time_delta = common::ToSeconds(newer.time - older.time);
  if (time_delta < 1e-4) return;
  const transform::Rigid2d pose_delta = older.pose.inverse() * newer.pose;
  angular_velocity_from_odometry_ = transform::Yaw(pose_delta) / time_delta;
  if (timed_pose_queue_.empty()) return;

  const Eigen::Vector2d velocity_in_tracking_frame =
      pose_delta.translation() / time_delta;
  const double extrapolation_delta =
      common::ToSeconds(newer.time - timed_pose_queue_.back().time);
  const double orientation_at_newest =
      transform::Yaw(timed_pose_queue_.back().pose) +
      extrapolation_delta * angular_velocity_from_odometry_;
  linear_velocity_from_odometry_ =
      Eigen::Rotation2Dd(orientation_at_newest) * velocity_in_tracking_frame;
}

transform::Rigid2d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest = timed_pose_queue_.back();
  if (cached_extrapolated_pose_.time == time) {
    return cached_extrapolated_pose_.pose;
  }
  // Apply the odometry motion over [last pose, query] in the last SLAM frame.
  // A stale last-two-sample ω would keep spinning through a dropped scan.
  cached_extrapolated_pose_ = {
      time, newest.pose * TrackingMotion(time, newest.time)};
  return cached_extrapolated_pose_.pose;
}

transform::Rigid2d PoseExtrapolator::TrackingMotion(
    const common::Time time, const common::Time reference_time) const {
  transform::Rigid2d pose_at_time;
  transform::Rigid2d pose_at_reference;
  if (InterpolateOdometry(time, &pose_at_time) &&
      InterpolateOdometry(reference_time, &pose_at_reference)) {
    return pose_at_reference.inverse() * pose_at_time;
  }
  Eigen::Vector2d velocity_in_tracking = Eigen::Vector2d::Zero();
  double angular_velocity = 0.;
  CurrentBodyTwist(&velocity_in_tracking, &angular_velocity);
  return IntegrateBodyTwist(velocity_in_tracking, angular_velocity,
                            common::ToSeconds(time - reference_time));
}

bool PoseExtrapolator::InterpolateOdometry(
    const common::Time time, transform::Rigid2d* pose) const {
  if (odometry_data_.size() < 2 || pose == nullptr) return false;
  if (time + common::FromSeconds(kMaxOdometryExtrapolateSeconds) <
          odometry_data_.front().time ||
      odometry_data_.back().time +
              common::FromSeconds(kMaxOdometryExtrapolateSeconds) <
          time) {
    return false;
  }

  const auto later = std::lower_bound(
      odometry_data_.begin(), odometry_data_.end(), time,
      [](const sensor::OdometryData& data, const common::Time query) {
        return data.time < query;
      });
  const sensor::OdometryData* older = nullptr;
  const sensor::OdometryData* newer = nullptr;
  if (later == odometry_data_.begin()) {
    older = &odometry_data_.front();
    newer = &odometry_data_[1];
  } else if (later == odometry_data_.end()) {
    older = &odometry_data_[odometry_data_.size() - 2];
    newer = &odometry_data_.back();
  } else {
    newer = &(*later);
    older = &(*std::prev(later));
  }

  const double duration = common::ToSeconds(newer->time - older->time);
  if (duration < 1e-6) {
    *pose = newer->pose;
    return true;
  }
  const double factor = common::ToSeconds(time - older->time) / duration;
  const Eigen::Vector2d translation =
      older->pose.translation() +
      factor * (newer->pose.translation() - older->pose.translation());
  const double yaw =
      transform::Yaw(older->pose) +
      factor * common::NormalizeAngleDifference(transform::Yaw(newer->pose) -
                                                transform::Yaw(older->pose));
  *pose = transform::MakeRigid2(translation, yaw);
  return true;
}

void PoseExtrapolator::CurrentBodyTwist(
    Eigen::Vector2d* linear_velocity_in_tracking,
    double* angular_velocity) const {
  CHECK(linear_velocity_in_tracking != nullptr);
  CHECK(angular_velocity != nullptr);
  if (odometry_data_.size() >= 2) {
    const auto& older = odometry_data_[odometry_data_.size() - 2];
    const auto& newer = odometry_data_.back();
    const double time_delta = common::ToSeconds(newer.time - older.time);
    if (time_delta >= 1e-4) {
      const transform::Rigid2d pose_delta = older.pose.inverse() * newer.pose;
      *linear_velocity_in_tracking = pose_delta.translation() / time_delta;
      *angular_velocity = transform::Yaw(pose_delta) / time_delta;
      return;
    }
  }
  if (timed_pose_queue_.size() >= 2) {
    const TimedPose& oldest = timed_pose_queue_.front();
    const TimedPose& newest = timed_pose_queue_.back();
    const double time_delta = common::ToSeconds(newest.time - oldest.time);
    if (time_delta >= 1e-3) {
      *angular_velocity =
          transform::Yaw(oldest.pose.inverse() * newest.pose) / time_delta;
      *linear_velocity_in_tracking =
          Eigen::Rotation2Dd(-transform::Yaw(newest.pose)) *
          ((newest.pose.translation() - oldest.pose.translation()) /
           time_delta);
      return;
    }
  }
  *linear_velocity_in_tracking = Eigen::Vector2d::Zero();
  *angular_velocity = 0.;
}

void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) return;
  const TimedPose& oldest = timed_pose_queue_.front();
  const TimedPose& newest = timed_pose_queue_.back();
  const double delta = common::ToSeconds(newest.time - oldest.time);
  if (delta < 1e-3) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << delta << " s";
    return;
  }
  linear_velocity_from_poses_ =
      (newest.pose.translation() - oldest.pose.translation()) / delta;
  angular_velocity_from_poses_ =
      transform::Yaw(oldest.pose.inverse() * newest.pose) / delta;
}

void PoseExtrapolator::TrimOdometryData() {
  const common::Time keep_after =
      timed_pose_queue_.empty()
          ? common::Time::min()
          : timed_pose_queue_.back().time -
                common::FromSeconds(kOdometryHistorySeconds);
  while (odometry_data_.size() > 2 && odometry_data_[1].time <= keep_after) {
    odometry_data_.pop_front();
  }
}

}  // namespace mapping
}  // namespace cartographer
