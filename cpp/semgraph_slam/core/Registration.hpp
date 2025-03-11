/* 
    SemGraphSLAM is heavily inspired by the framework of KISS-ICP (https://github.com/PRBonn/kiss-icp).
    We are deeply appreciative of Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss for their contributions to the open-source community.
*/

// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>
#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>
#include <unordered_map>

#include "VoxelHashMap.hpp"
#include "Coreutils.h"

namespace  {

// label-specifc weights for residual optimization
std::unordered_map<int, double> weights_semantic={
  {0, 1},     // "unlabeled"
  {1, 1},     // "outlier" mapped to "unlabeled" --------------------------mapped
  {1, 1},     // "car"
  {2, 1},     // "bicycle"
  {5, 1},     // "bus" mapped to "other-vehicle" --------------------------mapped
  {3, 1},     // "motorcycle"
  {5, 1},     // "on-rails" mapped to "other-vehicle" ---------------------mapped
  {4, 1},     // "truck"
  {5, 1},     // "other-vehicle"
  {6, 1},     // "person"
  {7, 1},     // "bicyclist"
  {8, 1},     // "motorcyclist"
  {9, 1},     // "road"
  {10, 1},    // "parking"
  {11, 1},    // "sidewalk"
  {12, 1},    // "other-ground"
  {13, 1},    // "building"
  {14, 1},    // "fence"
  {0, 1},     // "other-structure" mapped to "unlabeled" ------------------mapped
  {9, 1},     // "lane-marking" to "road" ---------------------------------mapped
  {15, 1},    // "vegetation"
  {16, 1.2},    // "trunk"
  {17, 1},    // "terrain"
  {18, 1.2},    // "pole"
  {19, 1.2},    // "traffic-sign"
  {0, 1},     // "other-object" to "unlabeled" ----------------------------mapped
  {20, 0},    // "moving-car"
  {21, 0},    // "moving-bicyclist"
  {22, 0},    // "moving-person"
  {23, 0},    // "moving-motorcyclist"
  {24, 0},    // "moving-on-rails" mapped to "moving-other-vehicle" ------mapped
  {24, 0},    // "moving-bus" mapped to "moving-other-vehicle" -----------mapped
  {25, 0},    // "moving-truck"
  {24, 0}    // "moving-other-vehicle
}; 

}

namespace graph_slam {

Sophus::SE3d RegisterFrameSemantic(const std::vector<Eigen::Vector4d> &frame,
                           const VoxelHashMap &voxel_map,
                           const Sophus::SE3d &initial_guess,
                           double max_correspondence_distance,
                           double kernel);
}   // namespace graph_slam
