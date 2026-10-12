# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)

project(cartographer_bag_application LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

get_filename_component(CARTOGRAPHER_SOURCE_PARENT
  "${PROJECT_SOURCE_DIR}/.." ABSOLUTE)

find_package(absl REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Sophus 1.22.10 CONFIG REQUIRED)
find_package(gflags REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

set(ALL_LIBRARY_SRCS
  frontend/data_dispatcher.cc
  foundation/sampling.cc
  application/config.cc
  foundation/sensor_data.cc
  foundation/voxel_filter.cc
  scan_matching/ceres_scan_matcher_2d.cc
  frontend/frontend_2d.cc
  backend/constraint_engine_2d.cc
  scan_matching/correlative_scan_matcher_2d.cc
  scan_matching/fast_correlative_scan_matcher_2d.cc
  scan_matching/real_time_correlative_scan_matcher_2d.cc
  mapping/grid_2d.cc
  frontend/local_slam_2d.cc
  application/slam_system.cc
  frontend/motion_filter.cc
  backend/pose_optimizer_2d.cc
  backend/ceres_pose_optimizer_2d.cc
  backend/gtsam_pose_optimizer_2d.cc
  backend/gtsam_isam2_pose_optimizer_2d.cc
  application/slam_options.cc
  backend/overlapping_submaps_trimmer_2d.cc
  frontend/pose_extrapolator.cc
  backend/trajectory_backend_2d.cc
  backend/trajectory_backend_query_2d.cc
  backend/backend_trimmer.cc
  mapping/submap_2d.cc
  serialization/swmap.cc
)

add_library(cartographer_offline_core STATIC ${ALL_LIBRARY_SRCS})

target_include_directories(cartographer_offline_core SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(cartographer_offline_core PUBLIC ${EIGEN3_LIBRARIES})
target_link_libraries(cartographer_offline_core PUBLIC Sophus::Sophus)

target_include_directories(cartographer_offline_core SYSTEM PUBLIC
  "${CERES_INCLUDE_DIRS}")
target_link_libraries(cartographer_offline_core PUBLIC ${CERES_LIBRARIES})
target_link_libraries(cartographer_offline_core PUBLIC gtsam)

target_link_libraries(cartographer_offline_core PUBLIC yaml-cpp)

if (WIN32)
  find_package(glog REQUIRED)
  set(GLOG_LIBRARY glog::glog)
else()
  set(GLOG_LIBRARY glog)
endif()

target_link_libraries(cartographer_offline_core PUBLIC ${GLOG_LIBRARY})
target_link_libraries(cartographer_offline_core PUBLIC gflags)
if(WIN32)
  # Needed to fix conflict with MSVC's error macro.
  target_compile_definitions(cartographer_offline_core PUBLIC -DGLOG_NO_ABBREVIATED_SEVERITIES)
endif()
if(MSVC)
  # Needed for VS 2017 5.8
  target_compile_definitions(cartographer_offline_core PUBLIC -D_ENABLE_EXTENDED_ALIGNED_STORAGE -D_USE_MATH_DEFINES)
endif()

target_link_libraries(cartographer_offline_core PUBLIC sqlite3 ZLIB::ZLIB
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility 
)
if (NOT WIN32)
  target_link_libraries(cartographer_offline_core PUBLIC pthread)
endif()
target_include_directories(cartographer_offline_core PUBLIC
  "${CARTOGRAPHER_SOURCE_PARENT}"
)

if(NOT WIN32)
  target_compile_options(cartographer_offline_core PRIVATE
    -fPIC -Wall -Wpedantic
    -Werror=format-security
    -Werror=missing-braces
    -Werror=reorder
    -Werror=return-type
    -Werror=switch
    -Werror=uninitialized
  )
endif()

add_executable(cartographer_bag_runner application/bag_runner.cc)
target_link_libraries(cartographer_bag_runner PRIVATE
  cartographer_offline_core
  gflags
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS cartographer_bag_runner RUNTIME DESTINATION bin)
