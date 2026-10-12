// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include <sys/resource.h>
#include <time.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "cartographer/application/config.h"
#include "cartographer/foundation/time.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/foundation/transform.h"
#include "cartographer/application/slam_system.h"
#include "cartographer/backend/backend_types.h"
#include "cartographer/frontend/frontend_2d.h"
#include "cartographer/mapping/grid_2d.h"
#include "cartographer/mapping/submap_2d.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "yaml-cpp/yaml.h"

DEFINE_string(offline_configuration_directory, "", "Configuration directory");
DEFINE_string(configuration_basenames, "", "Single YAML configuration basename");
DEFINE_string(bag_filenames, "", "Single ROS 2 bag directory");
DEFINE_string(calibration_filename, "", "Sensor calibration YAML");
DEFINE_string(offline_load_state_filename, "", "Frozen .swmap input");
DEFINE_bool(offline_load_frozen_state, true, "Load map as frozen");
DEFINE_string(offline_save_state_filename, "", ".swmap output");
DEFINE_string(trajectory_filename, "", "Optimized trajectory CSV output");
DEFINE_int32(debug_max_scan_count, -1,
             "Process only the first N lidar scans for prefix debugging");
DEFINE_string(debug_step_directory, "",
              "If set, dump per-scan pose, hits, and occupancy after each "
              "LocalSlam step");

namespace {
namespace carto = cartographer;

carto::common::Time FromRos(const builtin_interfaces::msg::Time& stamp) {
  const int64_t nanoseconds = static_cast<int64_t>(stamp.sec) * 1000000000ll +
                              stamp.nanosec;
  return carto::common::FromUniversal(
      carto::common::kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll +
      (nanoseconds + 50) / 100);
}

double ToUnixSeconds(carto::common::Time time) {
  return (carto::common::ToUniversal(time) -
          carto::common::kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) *
         1e-7;
}

carto::transform::Rigid2d ToRigid2d(const geometry_msgs::msg::Pose& pose) {
  const Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x,
                                    pose.orientation.y, pose.orientation.z);
  CHECK(std::isfinite(pose.position.x) && std::isfinite(pose.position.y));
  CHECK(rotation.coeffs().allFinite());
  CHECK_GT(rotation.norm(), 1e-9);
  const Eigen::Quaterniond normalized = rotation.normalized();
  const double yaw = std::atan2(
      2. * (normalized.w() * normalized.z() +
            normalized.x() * normalized.y()),
      1. - 2. * (normalized.y() * normalized.y() +
                 normalized.z() * normalized.z()));
  return carto::transform::MakeRigid2(
      Eigen::Vector2d(pose.position.x, pose.position.y), yaw);
}

std::string NormalizeFrame(std::string frame) {
  if (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
  return frame;
}

template <typename Message>
Message Deserialize(const rosbag2_storage::SerializedBagMessage& message) {
  rclcpp::SerializedMessage serialized(*message.serialized_data);
  Message result;
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &result);
  return result;
}

std::map<std::string, std::string> TopicTypes(rosbag2_cpp::Reader* reader) {
  std::map<std::string, std::string> result;
  for (const auto& topic : reader->get_all_topics_and_types()) {
    result.emplace(topic.name, topic.type);
  }
  return result;
}

struct Calibration {
  std::string tracking_frame;
  std::string lidar_topic;
  std::string lidar_frame;
  double lidar_time_offset_seconds;
  bool lidar_motion_compensation;
  bool lidar_stamp_at_end;
  double lidar_scan_period_seconds;
  carto::transform::Rigid2d tracking_from_lidar;
  std::string odometry_topic;
  std::string odometry_reference_frame;
  std::string odometry_child_frame;
  double odometry_time_offset_seconds;
  carto::transform::Rigid2d tracking_from_odometry_child;
};

carto::transform::Rigid2d ReadPlanarTransform(const YAML::Node& node,
                                              const std::string& name) {
  CHECK(node.IsMap()) << name << " must contain x, y and yaw.";
  const double x = node["x"].as<double>();
  const double y = node["y"].as<double>();
  const double yaw = node["yaw"].as<double>();
  CHECK(std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw))
      << name << " is not finite.";
  return carto::transform::MakeRigid2(Eigen::Vector2d(x, y), yaw);
}

Calibration LoadCalibration(const std::string& filename) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(filename);
  } catch (const std::exception& error) {
    LOG(FATAL) << "Failed to load calibration '" << filename
               << "': " << error.what();
  }
  CHECK_EQ(root["schema_version"].as<int>(), 2);
  CHECK_EQ(root["convention"].as<std::string>(), "T_parent_child");
  CHECK_EQ(root["units"]["translation"].as<std::string>(), "meter");
  CHECK_EQ(root["units"]["rotation"].as<std::string>(), "radian");
  const auto lidar = root["lidar"];
  const auto odometry = root["odometry"];
  CHECK_EQ(lidar["message_type"].as<std::string>(),
           "sensor_msgs/msg/LaserScan");
  CHECK_EQ(odometry["message_type"].as<std::string>(),
           "nav_msgs/msg/Odometry");
  CHECK_EQ(odometry["pose_convention"].as<std::string>(),
           "T_reference_child");
  const double lidar_time_offset_seconds =
      lidar["time_offset_seconds"].as<double>();
  const bool lidar_motion_compensation =
      !lidar["motion_compensation"].IsDefined() ||
      lidar["motion_compensation"].as<bool>();
  const std::string lidar_stamp_convention =
      lidar["stamp_convention"].IsDefined()
          ? lidar["stamp_convention"].as<std::string>()
          : "start";
  CHECK(lidar_stamp_convention == "start" ||
        lidar_stamp_convention == "end")
      << "lidar.stamp_convention must be 'start' or 'end'";
  const bool lidar_stamp_at_end = lidar_stamp_convention == "end";
  const double lidar_scan_period_seconds =
      lidar["scan_period_seconds"].IsDefined()
          ? lidar["scan_period_seconds"].as<double>()
          : 0.0;
  CHECK(std::isfinite(lidar_scan_period_seconds) &&
        lidar_scan_period_seconds >= 0.0);
  const double odometry_time_offset_seconds =
      odometry["time_offset_seconds"].as<double>();
  CHECK(std::isfinite(lidar_time_offset_seconds));
  CHECK(std::isfinite(odometry_time_offset_seconds));
  if (lidar_time_offset_seconds != 0. || odometry_time_offset_seconds != 0.) {
    LOG(INFO) << "Using calibrated sensor time offsets: lidar="
              << lidar_time_offset_seconds << " s, odometry="
              << odometry_time_offset_seconds
              << " s (corrected_time = message_time + offset).";
  }
  return {
      NormalizeFrame(root["tracking_frame"].as<std::string>()),
      lidar["topic"].as<std::string>(),
      NormalizeFrame(lidar["frame"].as<std::string>()),
      lidar_time_offset_seconds,
      lidar_motion_compensation,
      lidar_stamp_at_end,
      lidar_scan_period_seconds,
      ReadPlanarTransform(lidar["T_tracking_sensor"], "T_tracking_sensor"),
      odometry["topic"].as<std::string>(),
      NormalizeFrame(odometry["reference_frame"].as<std::string>()),
      NormalizeFrame(odometry["child_frame"].as<std::string>()),
      odometry_time_offset_seconds,
      ReadPlanarTransform(odometry["T_tracking_child"], "T_tracking_child")};
}

bool CanMotionCompensate(const sensor_msgs::msg::LaserScan& scan,
                         const Calibration& calibration) {
  return calibration.lidar_motion_compensation &&
         std::isfinite(scan.time_increment) && scan.time_increment > 1e-6 &&
         scan.ranges.size() > 1;
}

bool IsUnusableLaserScan(const sensor_msgs::msg::LaserScan& scan,
                         const Calibration& calibration) {
  const double expected = calibration.lidar_scan_period_seconds;
  if (expected <= 0. || !std::isfinite(scan.scan_time) || scan.scan_time <= 0.) {
    return false;
  }
  // LD14-only: calibration.scan_period_seconds must be set. Gazebo omits it
  // (expected == 0) so this never drops simulation scans.
  return scan.scan_time > 1.5 * expected || scan.scan_time < 0.5 * expected;
}

carto::sensor::TimedPointCloudData ConvertScan(
    const sensor_msgs::msg::LaserScan& scan,
    const Calibration& calibration) {
  CHECK_EQ(NormalizeFrame(scan.header.frame_id), calibration.lidar_frame);
  carto::sensor::TimedPointCloud points;
  points.reserve(scan.ranges.size());
  const bool use_motion_compensation = CanMotionCompensate(scan, calibration);
  const float increment = use_motion_compensation ? scan.time_increment : 0.f;
  float angle = scan.angle_min;
  for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
    const float range = scan.ranges[i];
    if (std::isfinite(range) && range >= scan.range_min &&
        range <= scan.range_max) {
      points.push_back({{range * std::cos(angle), range * std::sin(angle), 0.f},
                        static_cast<float>(i) * increment});
    }
  }
  auto time = FromRos(scan.header.stamp);
  time += carto::common::FromSeconds(calibration.lidar_time_offset_seconds);
  if (use_motion_compensation && !points.empty()) {
    const float last_point_time = points.back().time;
    for (auto& point : points) point.time -= last_point_time;
    if (!calibration.lidar_stamp_at_end) {
      time += carto::common::FromSeconds(last_point_time);
    }
  }
  const auto sensor_to_tracking = carto::transform::Embed3D(
      calibration.tracking_from_lidar.cast<float>());
  return {time, sensor_to_tracking.translation(),
          carto::sensor::TransformTimedPointCloud(points, sensor_to_tracking),
          {}};
}

carto::sensor::OdometryData ConvertOdometry(
    const nav_msgs::msg::Odometry& odometry,
    const Calibration& calibration) {
  CHECK_EQ(NormalizeFrame(odometry.header.frame_id),
           calibration.odometry_reference_frame);
  CHECK_EQ(NormalizeFrame(odometry.child_frame_id),
           calibration.odometry_child_frame);
  return {FromRos(odometry.header.stamp) +
              carto::common::FromSeconds(
                  calibration.odometry_time_offset_seconds),
          ToRigid2d(odometry.pose.pose) *
              calibration.tracking_from_odometry_child.inverse()};
}

void ValidateBagOnlyConfig(
    carto::common::ParameterDictionary* dictionary,
    const carto::mapping::TrajectoryBuilderOptions& trajectory_options) {
  if (dictionary->HasKey("num_laser_scans"))
    CHECK_EQ(dictionary->GetNonNegativeInt("num_laser_scans"), 1);
  if (dictionary->HasKey("num_multi_echo_laser_scans"))
    CHECK_EQ(dictionary->GetNonNegativeInt("num_multi_echo_laser_scans"), 0)
      << "MultiEchoLaserScan is not supported by the bag-only application.";
  if (dictionary->HasKey("num_point_clouds"))
    CHECK_EQ(dictionary->GetNonNegativeInt("num_point_clouds"), 0)
      << "PointCloud2 is not supported by the bag-only application.";
  if (dictionary->HasKey("use_nav_sat"))
    CHECK(!dictionary->GetBool("use_nav_sat"))
      << "GPS is not supported by the bag-only application.";
  if (dictionary->HasKey("use_landmarks"))
    CHECK(!dictionary->GetBool("use_landmarks"))
      << "Landmarks are not supported by the bag-only application.";
}

void WriteTrajectory(const std::string& filename, int trajectory_id,
                     carto::mapping::TrajectoryBackend2D* backend) {
  if (filename.empty()) return;
  std::ofstream output(filename);
  CHECK(output.good()) << "Cannot open trajectory output '" << filename << "'.";
  output << std::setprecision(17) << "timestamp,x,y,theta\n";
  for (const auto& item : backend->GetTrajectoryNodes()) {
    if (item.id.trajectory_id != trajectory_id) continue;
    const auto& pose = item.data.global_pose;
    const double yaw = carto::transform::Yaw(pose);
    output << ToUnixSeconds(item.data.time()) << ',' << pose.translation().x()
           << ',' << pose.translation().y() << ',' << yaw << '\n';
  }
}

struct OccupancyDump {
  bool empty = true;
  int width = 0;
  int height = 0;
  double origin_x = 0.;
  double origin_y = 0.;
  double resolution = 0.05;
  std::string filename;
};

struct DebugFrame {
  double t = 0.;
  double x = 0.;
  double y = 0.;
  double yaw = 0.;
  bool inserted = false;
  OccupancyDump map;
  std::vector<Eigen::Vector2d> hits;
  std::vector<Eigen::Vector2d> rejected_hits;
};

OccupancyDump WriteOccupancy(const std::string& directory, int index,
                             carto::mapping::TrajectoryBackend2D* backend) {
  double resolution = 0.03;
  constexpr double padding = 0.25;
  constexpr float kFree = 0.48f;
  constexpr float kOccupied = 0.52f;

  struct Cell {
    Eigen::Vector2d local;
    float probability;
    carto::transform::Rigid2d global_from_local;
  };
  std::vector<Cell> cells;
  Eigen::Vector2d minimum = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector2d maximum = Eigen::Vector2d::Constant(
      -std::numeric_limits<double>::infinity());

  for (const auto& item : backend->GetAllSubmapData()) {
    const auto submap =
        std::dynamic_pointer_cast<const carto::mapping::Submap2D>(
            item.data.submap);
    if (!submap || submap->grid() == nullptr) continue;
    const auto* grid = dynamic_cast<const carto::mapping::ProbabilityGrid*>(
        submap->grid());
    if (grid == nullptr) continue;
    resolution = std::min(resolution, grid->limits().resolution());
    const auto global_from_local =
        item.data.pose * submap->local_pose().inverse();
    for (const auto& xy : carto::mapping::XYIndexRangeIterator(
             grid->limits().cell_limits())) {
      if (!grid->IsKnown(xy)) continue;
      const Eigen::Vector2d local =
          grid->limits().GetCellCenter(xy).cast<double>();
      const Eigen::Vector2d global = global_from_local * local;
      minimum = minimum.cwiseMin(global);
      maximum = maximum.cwiseMax(global);
      cells.push_back({local, grid->GetProbability(xy), global_from_local});
    }
  }

  OccupancyDump dump;
  if (cells.empty() || !minimum.allFinite()) return dump;
  minimum.array() -= padding;
  maximum.array() += padding;
  dump.width = std::max(
      1, static_cast<int>(std::ceil((maximum.x() - minimum.x()) / resolution)));
  dump.height = std::max(
      1, static_cast<int>(std::ceil((maximum.y() - minimum.y()) / resolution)));
  if (dump.width > 2000 || dump.height > 2000) {
    LOG(WARNING) << "skipping oversized debug map " << dump.width << "x"
                 << dump.height;
    return dump;
  }
  dump.origin_x = minimum.x();
  dump.origin_y = minimum.y();
  dump.resolution = resolution;
  dump.empty = false;
  std::vector<uint8_t> pixels(
      static_cast<size_t>(dump.width) * static_cast<size_t>(dump.height), 128);
  for (const auto& cell : cells) {
    const Eigen::Vector2d global = cell.global_from_local * cell.local;
    const int x = carto::common::RoundToInt(
        (global.x() - minimum.x()) / resolution);
    const int y = carto::common::RoundToInt(
        (global.y() - minimum.y()) / resolution);
    if (x < 0 || y < 0 || x >= dump.width || y >= dump.height) continue;
    uint8_t gray = 128;
    if (cell.probability <= kFree) {
      gray = 255;
    } else if (cell.probability >= kOccupied) {
      gray = 0;
    } else {
      continue;
    }
    pixels[(dump.height - 1 - y) * dump.width + x] = gray;
  }
  char name[64];
  std::snprintf(name, sizeof(name), "maps/%06d.pgm", index);
  dump.filename = name;
  const std::string path = directory + "/" + dump.filename;
  std::ofstream out(path, std::ios::binary);
  CHECK(out) << "Cannot write occupancy '" << path << "'.";
  out << "P5\n" << dump.width << " " << dump.height << "\n255\n";
  out.write(reinterpret_cast<const char*>(pixels.data()),
            static_cast<std::streamsize>(pixels.size()));
  CHECK(out) << "Failed writing occupancy '" << path << "'.";
  return dump;
}

void WriteDebugFrames(const std::string& directory,
                      const std::vector<DebugFrame>& frames) {
  if (directory.empty()) return;
  const std::string path = directory + "/frames.json";
  std::ofstream out(path);
  CHECK(out) << "Cannot write debug frames '" << path << "'.";
  out << std::fixed << "{\"frames\":[\n";
  for (size_t i = 0; i < frames.size(); ++i) {
    const auto& frame = frames[i];
    out << " {\"t\":" << std::setprecision(6) << frame.t
        << ",\"x\":" << std::setprecision(4) << frame.x
        << ",\"y\":" << frame.y
        << ",\"yaw\":" << frame.yaw
        << ",\"inserted\":" << (frame.inserted ? "true" : "false")
        << ",\"map\":\"" << frame.map.filename << "\""
        << ",\"origin_x\":" << frame.map.origin_x
        << ",\"origin_y\":" << frame.map.origin_y
        << ",\"resolution\":" << frame.map.resolution
        << ",\"width\":" << frame.map.width
        << ",\"height\":" << frame.map.height
        << ",\"hits\":[";
    for (size_t j = 0; j < frame.hits.size(); ++j) {
      if (j != 0) out << ",";
      out << "[" << std::setprecision(3) << frame.hits[j].x() << ","
          << frame.hits[j].y() << "]";
    }
    out << "],\"rejected_hits\":[";
    for (size_t j = 0; j < frame.rejected_hits.size(); ++j) {
      if (j != 0) out << ",";
      out << "[" << std::setprecision(3) << frame.rejected_hits[j].x() << ","
          << frame.rejected_hits[j].y() << "]";
    }
    out << "]}";
    if (i + 1 < frames.size()) out << ",";
    out << "\n";
  }
  out << "]}\n";
  CHECK(out);
  LOG(INFO) << "Wrote " << frames.size() << " debug steps to " << directory;
}

using ConstraintKey = std::array<int, 4>;

ConstraintKey MakeConstraintKey(const carto::mapping::Constraint& c) {
  return {c.submap_id.trajectory_id, c.submap_id.submap_index,
          c.node_id.trajectory_id, c.node_id.node_index};
}

struct PoseGraphNodeDump {
  int tid = 0;
  int nid = 0;
  double t = 0.;
  double x = 0.;
  double y = 0.;
  double yaw = 0.;
};

struct PoseGraphSubmapDump {
  int tid = 0;
  int sid = 0;
  double x = 0.;
  double y = 0.;
  double yaw = 0.;
};

struct PoseGraphConstraintDump {
  int submap_tid = 0;
  int submap_sid = 0;
  int node_tid = 0;
  int node_nid = 0;
  std::string tag;
  double tx = 0.;
  double ty = 0.;
  double tyaw = 0.;
};

struct PoseGraphNodeDelta {
  int tid = 0;
  int nid = 0;
  double dx = 0.;
  double dy = 0.;
  double d_m = 0.;
};

struct PoseGraphOptEvent {
  int index = 0;
  int frame = -1;
  double t = 0.;
  int inter_total = 0;
  int intra_total = 0;
  double max_delta_m = 0.;
  std::vector<ConstraintKey> new_inter;
  std::vector<PoseGraphNodeDelta> node_deltas;
  std::vector<PoseGraphNodeDump> nodes;
  std::vector<PoseGraphSubmapDump> submaps;
  std::vector<PoseGraphConstraintDump> constraints;
};

struct PoseGraphDump {
  std::vector<PoseGraphNodeDump> nodes;
  std::vector<PoseGraphSubmapDump> submaps;
  std::vector<PoseGraphConstraintDump> constraints;
  std::vector<PoseGraphOptEvent> events;
};

std::vector<PoseGraphNodeDump> CollectNodes(
    carto::mapping::TrajectoryBackend2D* backend) {
  std::vector<PoseGraphNodeDump> nodes;
  for (const auto& item : backend->GetTrajectoryNodes()) {
    PoseGraphNodeDump node;
    node.tid = item.id.trajectory_id;
    node.nid = item.id.node_index;
    node.t = ToUnixSeconds(item.data.time());
    node.x = item.data.global_pose.translation().x();
    node.y = item.data.global_pose.translation().y();
    node.yaw = carto::transform::Yaw(item.data.global_pose);
    nodes.push_back(node);
  }
  return nodes;
}

std::vector<PoseGraphSubmapDump> CollectSubmaps(
    carto::mapping::TrajectoryBackend2D* backend) {
  std::vector<PoseGraphSubmapDump> submaps;
  for (const auto& item : backend->GetAllSubmapData()) {
    PoseGraphSubmapDump submap;
    submap.tid = item.id.trajectory_id;
    submap.sid = item.id.submap_index;
    submap.x = item.data.pose.translation().x();
    submap.y = item.data.pose.translation().y();
    submap.yaw = carto::transform::Yaw(item.data.pose);
    submaps.push_back(submap);
  }
  return submaps;
}

std::vector<PoseGraphConstraintDump> CollectConstraints(
    carto::mapping::TrajectoryBackend2D* backend, int* inter_total,
    int* intra_total, std::set<ConstraintKey>* inter_keys) {
  std::vector<PoseGraphConstraintDump> constraints;
  int inter = 0;
  int intra = 0;
  for (const auto& c : backend->constraints()) {
    PoseGraphConstraintDump dump;
    dump.submap_tid = c.submap_id.trajectory_id;
    dump.submap_sid = c.submap_id.submap_index;
    dump.node_tid = c.node_id.trajectory_id;
    dump.node_nid = c.node_id.node_index;
    dump.tag = c.tag == carto::mapping::Constraint::INTER_SUBMAP ? "INTER"
                                                                 : "INTRA";
    dump.tx = c.pose.zbar_ij.translation().x();
    dump.ty = c.pose.zbar_ij.translation().y();
    dump.tyaw = carto::transform::Yaw(c.pose.zbar_ij);
    if (c.tag == carto::mapping::Constraint::INTER_SUBMAP) {
      ++inter;
      if (inter_keys != nullptr) inter_keys->insert(MakeConstraintKey(c));
    } else {
      ++intra;
    }
    constraints.push_back(dump);
  }
  if (inter_total != nullptr) *inter_total = inter;
  if (intra_total != nullptr) *intra_total = intra;
  return constraints;
}

void WriteConstraintJson(std::ostream& out, const PoseGraphConstraintDump& c) {
  out << "{\"submap\":[" << c.submap_tid << "," << c.submap_sid << "],\"node\":["
      << c.node_tid << "," << c.node_nid << "],\"tag\":\"" << c.tag << "\""
      << ",\"tx\":" << std::setprecision(4) << c.tx << ",\"ty\":" << c.ty
      << ",\"tyaw\":" << c.tyaw << "}";
}

void WriteNodeJson(std::ostream& out, const PoseGraphNodeDump& n) {
  out << "{\"tid\":" << n.tid << ",\"nid\":" << n.nid
      << ",\"t\":" << std::setprecision(6) << n.t
      << ",\"x\":" << std::setprecision(4) << n.x << ",\"y\":" << n.y
      << ",\"yaw\":" << n.yaw << "}";
}

void WriteSubmapJson(std::ostream& out, const PoseGraphSubmapDump& s) {
  out << "{\"tid\":" << s.tid << ",\"sid\":" << s.sid
      << ",\"x\":" << std::setprecision(4) << s.x << ",\"y\":" << s.y
      << ",\"yaw\":" << s.yaw << "}";
}

void WritePoseGraph(const std::string& directory, const PoseGraphDump& dump) {
  if (directory.empty()) return;
  const std::string path = directory + "/pose_graph.json";
  std::ofstream out(path);
  CHECK(out) << "Cannot write pose graph '" << path << "'.";
  out << std::fixed << "{\n\"nodes\":[\n";
  for (size_t i = 0; i < dump.nodes.size(); ++i) {
    if (i != 0) out << ",\n";
    out << " ";
    WriteNodeJson(out, dump.nodes[i]);
  }
  out << "\n],\"submaps\":[\n";
  for (size_t i = 0; i < dump.submaps.size(); ++i) {
    if (i != 0) out << ",\n";
    out << " ";
    WriteSubmapJson(out, dump.submaps[i]);
  }
  out << "\n],\"constraints\":[\n";
  for (size_t i = 0; i < dump.constraints.size(); ++i) {
    if (i != 0) out << ",\n";
    out << " ";
    WriteConstraintJson(out, dump.constraints[i]);
  }
  out << "\n],\"events\":[\n";
  for (size_t i = 0; i < dump.events.size(); ++i) {
    const auto& event = dump.events[i];
    if (i != 0) out << ",\n";
    out << " {\"index\":" << event.index << ",\"frame\":" << event.frame
        << ",\"t\":" << std::setprecision(6) << event.t
        << ",\"inter_total\":" << event.inter_total
        << ",\"intra_total\":" << event.intra_total
        << ",\"max_delta_m\":" << std::setprecision(4) << event.max_delta_m
        << ",\"new_inter\":[";
    for (size_t j = 0; j < event.new_inter.size(); ++j) {
      if (j != 0) out << ",";
      const auto& key = event.new_inter[j];
      out << "{\"submap\":[" << key[0] << "," << key[1] << "],\"node\":["
          << key[2] << "," << key[3] << "]}";
    }
    out << "],\"node_deltas\":[";
    for (size_t j = 0; j < event.node_deltas.size(); ++j) {
      if (j != 0) out << ",";
      const auto& d = event.node_deltas[j];
      out << "{\"tid\":" << d.tid << ",\"nid\":" << d.nid
          << ",\"dx\":" << std::setprecision(4) << d.dx << ",\"dy\":" << d.dy
          << ",\"d_m\":" << d.d_m << "}";
    }
    out << "],\"nodes\":[";
    for (size_t j = 0; j < event.nodes.size(); ++j) {
      if (j != 0) out << ",";
      WriteNodeJson(out, event.nodes[j]);
    }
    out << "],\"submaps\":[";
    for (size_t j = 0; j < event.submaps.size(); ++j) {
      if (j != 0) out << ",";
      WriteSubmapJson(out, event.submaps[j]);
    }
    out << "],\"constraints\":[";
    for (size_t j = 0; j < event.constraints.size(); ++j) {
      if (j != 0) out << ",";
      WriteConstraintJson(out, event.constraints[j]);
    }
    out << "]}";
  }
  out << "\n]}\n";
  CHECK(out);
  LOG(INFO) << "Wrote pose graph with " << dump.events.size()
            << " optimization events, " << dump.constraints.size()
            << " constraints to " << directory;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  CHECK(!FLAGS_offline_configuration_directory.empty());
  CHECK(!FLAGS_configuration_basenames.empty());
  CHECK(!FLAGS_bag_filenames.empty());
  CHECK(!FLAGS_calibration_filename.empty());
  CHECK_EQ(FLAGS_bag_filenames.find(','), std::string::npos)
      << "Only one bag is supported.";

  auto dictionary = carto::common::ParameterDictionary::LoadFile(
      FLAGS_offline_configuration_directory + "/" +
      FLAGS_configuration_basenames);
  const std::string tracking_frame =
      NormalizeFrame(dictionary->GetString("tracking_frame"));
  const Calibration calibration = LoadCalibration(FLAGS_calibration_filename);
  CHECK_EQ(calibration.tracking_frame, tracking_frame);
  CHECK(!calibration.lidar_topic.empty());
  CHECK(!calibration.odometry_topic.empty());
  CHECK_NE(calibration.lidar_topic, calibration.odometry_topic);
  const auto map_options = carto::mapping::CreateSlamSystemOptions(
      dictionary->GetDictionary("map_builder").get());
  const auto trajectory_options = carto::mapping::CreateTrajectoryBuilderOptions(
      dictionary->GetDictionary("trajectory_builder").get());
  ValidateBagOnlyConfig(dictionary.get(), trajectory_options);

  auto map_builder = carto::mapping::CreateSlamSystem(map_options);
  if (!FLAGS_offline_load_state_filename.empty()) {
    map_builder->LoadStateFromFile(FLAGS_offline_load_state_filename,
                                   FLAGS_offline_load_frozen_state);
  }
  const std::set<carto::mapping::SlamSystem::SensorId> sensors = {
      {carto::mapping::SlamSystem::SensorId::SensorType::RANGE,
       "scan"},
      {carto::mapping::SlamSystem::SensorId::SensorType::ODOMETRY,
       "odom"}};
  std::vector<DebugFrame> debug_frames;
  OccupancyDump last_map;
  std::mutex pose_graph_mutex;
  int latest_frame_index = -1;
  double latest_frame_t = 0.;
  PoseGraphDump pose_graph_dump;
  std::map<std::pair<int, int>, Eigen::Vector2d> previous_node_xy;
  std::set<ConstraintKey> previous_inter_keys;
  carto::mapping::SlamSystem::LocalSlamResultCallback debug_callback;
  if (!FLAGS_debug_step_directory.empty()) {
    std::filesystem::create_directories(
        std::filesystem::path(FLAGS_debug_step_directory) / "maps");
    debug_callback =
        [&](int trajectory_id, carto::common::Time time,
            carto::transform::Rigid2d local_pose,
            carto::sensor::RangeData range_data,
            std::unique_ptr<const carto::mapping::Frontend2D::InsertionResult>
                insertion,
            carto::sensor::PointCloud rejected_insert_hits) {
          const auto global_from_local =
              map_builder->backend()->GetLocalToGlobalTransform(trajectory_id);
          const auto pose = global_from_local * local_pose;
          DebugFrame frame;
          frame.t = ToUnixSeconds(time);
          frame.x = pose.translation().x();
          frame.y = pose.translation().y();
          frame.yaw = carto::transform::Yaw(pose);
          frame.inserted = insertion != nullptr;
          if (frame.inserted) {
            last_map = WriteOccupancy(FLAGS_debug_step_directory,
                                      static_cast<int>(debug_frames.size()),
                                      map_builder->backend());
          }
          frame.map = last_map;
          // Keep every return so the match pane can show the full sweep
          // (LD14 is ~600 beams; thinning to ~120 hid most of the cloud).
          const auto& points = range_data.returns.points();
          frame.hits.reserve(points.size());
          for (const auto& point : points) {
            const Eigen::Vector2d local(point.position.x(), point.position.y());
            frame.hits.push_back(global_from_local * local);
          }
          frame.rejected_hits.reserve(rejected_insert_hits.size());
          for (const auto& point : rejected_insert_hits) {
            const Eigen::Vector2d local(point.position.x(), point.position.y());
            frame.rejected_hits.push_back(global_from_local * local);
          }
          debug_frames.push_back(std::move(frame));
          {
            std::lock_guard<std::mutex> lock(pose_graph_mutex);
            latest_frame_index = static_cast<int>(debug_frames.size()) - 1;
            latest_frame_t = debug_frames.back().t;
          }
        };
    map_builder->backend()->SetGlobalSlamOptimizationCallback(
        [&](const std::map<int, carto::mapping::SubmapId>& /*submaps*/,
            const std::map<int, carto::mapping::NodeId>& /*nodes*/) {
          auto* backend = map_builder->backend();
          PoseGraphOptEvent event;
          {
            std::lock_guard<std::mutex> lock(pose_graph_mutex);
            event.index = static_cast<int>(pose_graph_dump.events.size());
            event.frame = latest_frame_index;
            event.t = latest_frame_t;
          }
          event.nodes = CollectNodes(backend);
          event.submaps = CollectSubmaps(backend);
          std::set<ConstraintKey> inter_keys;
          event.constraints = CollectConstraints(
              backend, &event.inter_total, &event.intra_total, &inter_keys);
          for (const auto& key : inter_keys) {
            if (previous_inter_keys.count(key) == 0) {
              event.new_inter.push_back(key);
            }
          }
          for (const auto& node : event.nodes) {
            const auto id = std::make_pair(node.tid, node.nid);
            const Eigen::Vector2d xy(node.x, node.y);
            const auto it = previous_node_xy.find(id);
            if (it == previous_node_xy.end()) continue;
            PoseGraphNodeDelta delta;
            delta.tid = node.tid;
            delta.nid = node.nid;
            delta.dx = xy.x() - it->second.x();
            delta.dy = xy.y() - it->second.y();
            delta.d_m = std::hypot(delta.dx, delta.dy);
            if (delta.d_m < 1e-4) continue;
            event.max_delta_m = std::max(event.max_delta_m, delta.d_m);
            event.node_deltas.push_back(delta);
          }
          previous_node_xy.clear();
          for (const auto& node : event.nodes) {
            previous_node_xy[std::make_pair(node.tid, node.nid)] =
                Eigen::Vector2d(node.x, node.y);
          }
          previous_inter_keys = std::move(inter_keys);
          {
            std::lock_guard<std::mutex> lock(pose_graph_mutex);
            pose_graph_dump.events.push_back(std::move(event));
          }
        });
  }
  const int trajectory_id = map_builder->AddTrajectoryBuilder(
      sensors, trajectory_options, debug_callback);
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'";

  const auto started = std::chrono::steady_clock::now();
  rosbag2_cpp::Reader reader;
  reader.open(FLAGS_bag_filenames);
  const auto types = TopicTypes(&reader);
  auto* trajectory = map_builder->GetTrajectoryBuilder(trajectory_id);
  struct QueuedMessage {
    int64_t stamp;
    rosbag2_storage::SerializedBagMessageSharedPtr message;
  };
  std::vector<QueuedMessage> queued;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    if (message->topic_name == calibration.lidar_topic) {
      CHECK_EQ(types.at(calibration.lidar_topic), "sensor_msgs/msg/LaserScan");
      const auto scan = Deserialize<sensor_msgs::msg::LaserScan>(*message);
      // Use the exact same conversion (including invalid-range filtering and
      // float rounding) as the dispatch path when repairing message order.
      queued.push_back({carto::common::ToUniversal(ConvertScan(scan, calibration).time),
                        message});
    } else if (message->topic_name == calibration.odometry_topic) {
      CHECK_EQ(types.at(calibration.odometry_topic), "nav_msgs/msg/Odometry");
      const auto odometry = Deserialize<nav_msgs::msg::Odometry>(*message);
      queued.push_back({carto::common::ToUniversal(
                            FromRos(odometry.header.stamp) +
                            carto::common::FromSeconds(
                                calibration.odometry_time_offset_seconds)),
                        message});
    }
  }
  std::stable_sort(queued.begin(), queued.end(),
                   [](const QueuedMessage& left, const QueuedMessage& right) {
                     return left.stamp < right.stamp;
                   });
  if (FLAGS_debug_max_scan_count >= 0) {
    int scan_count = 0;
    int64_t cutoff = std::numeric_limits<int64_t>::max();
    for (const auto& item : queued) {
      if (item.message->topic_name == calibration.lidar_topic) {
        ++scan_count;
        if (scan_count == FLAGS_debug_max_scan_count) {
          cutoff = item.stamp;
          break;
        }
      }
    }
    if (FLAGS_debug_max_scan_count == 0) cutoff = std::numeric_limits<int64_t>::min();
    std::vector<QueuedMessage> prefix;
    int kept_scans = 0;
    for (const auto& item : queued) {
      if (item.stamp > cutoff) continue;
      if (item.message->topic_name == calibration.lidar_topic &&
          ++kept_scans > FLAGS_debug_max_scan_count) continue;
      prefix.push_back(item);
    }
    queued.swap(prefix);
  }
  int dropped_abnormal_scans = 0;
  int motion_compensated_scans = 0;
  int64_t last_scan_stamp = std::numeric_limits<int64_t>::min();
  for (const auto& item : queued) {
    if (item.message->topic_name == calibration.lidar_topic) {
      const auto scan =
          Deserialize<sensor_msgs::msg::LaserScan>(*item.message);
      if (IsUnusableLaserScan(scan, calibration) ||
          item.stamp <= last_scan_stamp) {
        ++dropped_abnormal_scans;
        continue;
      }
      last_scan_stamp = item.stamp;
      if (CanMotionCompensate(scan, calibration)) ++motion_compensated_scans;
      trajectory->AddSensorData("scan", ConvertScan(scan, calibration));
    } else {
      const auto converted = ConvertOdometry(
          Deserialize<nav_msgs::msg::Odometry>(*item.message), calibration);
      trajectory->AddSensorData("odom", converted);
    }
  }
  if (dropped_abnormal_scans > 0) {
    LOG(WARNING) << "Dropped " << dropped_abnormal_scans
                 << " unusable or out-of-order lidar scans";
  }
  if (calibration.lidar_motion_compensation) {
    LOG(INFO) << "Motion-compensated " << motion_compensated_scans
              << " lidar scans using LaserScan.time_increment";
  }
  map_builder->FinishTrajectory(trajectory_id);
  map_builder->backend()->RunFinalOptimization();
  const double wall_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
          .count();
  LOG(INFO) << "Elapsed wall clock time: " << wall_seconds << " s";
  timespec cpu = {};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu);
  LOG(INFO) << "Elapsed CPU time: " << cpu.tv_sec + 1e-9 * cpu.tv_nsec << " s";
  rusage usage = {};
  CHECK_EQ(getrusage(RUSAGE_SELF, &usage), 0);
  LOG(INFO) << "Peak memory usage: " << usage.ru_maxrss << " KiB";
  WriteTrajectory(FLAGS_trajectory_filename, trajectory_id,
                  map_builder->backend());
  WriteDebugFrames(FLAGS_debug_step_directory, debug_frames);
  if (!FLAGS_debug_step_directory.empty()) {
    auto* backend = map_builder->backend();
    pose_graph_dump.nodes = CollectNodes(backend);
    pose_graph_dump.submaps = CollectSubmaps(backend);
    pose_graph_dump.constraints =
        CollectConstraints(backend, nullptr, nullptr, nullptr);
    WritePoseGraph(FLAGS_debug_step_directory, pose_graph_dump);
  }
  if (!FLAGS_offline_save_state_filename.empty()) {
    CHECK(map_builder->SerializeStateToFile(
        true, FLAGS_offline_save_state_filename));
  }
  rclcpp::shutdown();
  return 0;
}
