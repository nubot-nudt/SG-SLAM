/*
    The implementation of the PlaneSolver in this file is heavily inspired by the work STD and KISS-ICP
*/
// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tsl/robin_map.h>
#include <tbb/parallel_for.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "semgraph_slam/core/Coreutils.h"

namespace Eigen {
    using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen


namespace graph_slam{


struct PlaneSolver {
  PlaneSolver(Eigen::Vector3d curr_point_, Eigen::Vector3d curr_normal_,
              Eigen::Vector3d target_point_, Eigen::Vector3d target_normal_)
      : curr_point(curr_point_), curr_normal(curr_normal_),
        target_point(target_point_), target_normal(target_normal_){};
  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
                              T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;
    Eigen::Matrix<T, 3, 1> point_target(
        T(target_point.x()), T(target_point.y()), T(target_point.z()));
    Eigen::Matrix<T, 3, 1> norm(T(target_normal.x()), T(target_normal.y()),
                                T(target_normal.z()));
    residual[0] = norm.dot(point_w - point_target);
    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d curr_normal_,
                                     Eigen::Vector3d target_point_,
                                     Eigen::Vector3d target_normal_) {
    return (
        new ceres::AutoDiffCostFunction<PlaneSolver, 1, 4, 3>(new PlaneSolver(
            curr_point_, curr_normal_, target_point_, target_normal_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d curr_normal;
  Eigen::Vector3d target_point;
  Eigen::Vector3d target_normal;
};


class PlaneIcp
{
    private:
        // struct VoxelHash {
        // size_t operator()(const Eigen::Vector3i &voxel) const {
        //     const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        //     return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
        //     }
        // }; 
        struct VoxelBlock {
            std::vector<Eigen::Vector4d> voxel_points_;
        };
        struct VoxelBlock6d {
            std::vector<Eigen::Vector6d> voxel_points_;
        };
        /* data */
        double map_voxel_size_ = 0.5;
        double plane_thre_ = 0.005;
        int min_point_voxel_ = 8;
        double point_to_plane_thr_ = 0.5;
        double normal_thr_ = 0.2;
    public:
        PlaneIcp(double map_voxel_size);
        ~PlaneIcp();
        Eigen::Matrix4d getTransPlaneIcp(const V3d_i &cloud_a, const V3d_i &cloud_b, const Eigen::Matrix4d &init_trans);
        std::vector<Eigen::Vector6d> extractPlane(const V3d_i &cloud, double voxel_size);
        Eigen::Matrix4d PlaneAlign(const std::vector<Eigen::Vector6d> &cloud_a,
                                        const std::vector<Eigen::Vector6d> &cloud_b,
                                            const Eigen::MatrixX4d &init_trans, double voxel_size);
};


}