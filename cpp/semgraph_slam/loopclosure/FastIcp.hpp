/*
    This is simplified and modified from kiss-icp(https://github.com/PRBonn/kiss-icp.git)
*/
// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>


#pragma once

#define FMT_HEADER_ONLY
#include "fmt/format.h"

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>
#include <tsl/robin_map.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include "semgraph_slam/core/Coreutils.h"

namespace Eigen {
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

// using Vector4dVector = std::vector<Eigen::Vector4d>;

constexpr int MAX_NUM_ITERATIONS_ = 500;
constexpr double ESTIMATION_THRESHOLD_ = 0.0001;

namespace graph_slam{


using Vector4dVectorTuple = std::tuple<V4d, V4d>;

class FastIcp
{
    
private:
    /* data */
    // struct VoxelHash {
    // size_t operator()(const Eigen::Vector3i &voxel) const {
    //     const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
    //     return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    //     }
    // }; 
    struct VoxelBlock {
        // buffer of points with a max limit of n_points
        std::vector<Eigen::Vector4d> points;
        int num_points_;
        inline void AddPoint(const Eigen::Vector4d &point) {
                if (points.size() < static_cast<size_t>(num_points_)) points.push_back(point);
        }
    };

    struct ResultTuple {
        ResultTuple(std::size_t n) {
            source.reserve(n);
            target.reserve(n);
        }
        std::vector<Eigen::Vector4d> source;
        std::vector<Eigen::Vector4d> target;
    };
    struct ResultTupleRe {
        ResultTupleRe() {
            JTJ.setZero();
            JTr.setZero();
        }

        ResultTupleRe operator+(const ResultTupleRe &other) {
            this->JTJ += other.JTJ;
            this->JTr += other.JTr;
            return *this;
        }

        Eigen::Matrix6d JTJ;
        Eigen::Vector6d JTr;
    };

    std::vector<Eigen::Vector4d>  FusePointsAndLabels(const std::pair<std::vector<Eigen::Vector3d>,
                                                                    std::vector<int>> &frame);
    V3d_i SeparatePointsAndLabels(const std::vector<Eigen::Vector4d> &frame);
    Sophus::SE3d AlignClouds(const std::vector<Eigen::Vector4d> &source4d,
                         const std::vector<Eigen::Vector4d> &target4d);
    void TransformPoints4D(const Sophus::SE3d &T, std::vector<Eigen::Vector4d> &points);
    // voxel map
    float map_voel_size_ = 0.1;
    int max_points_per_voxel_ = 5;
    tsl::robin_map<Eigen::Vector3i, VoxelBlock, VoxelHash> map_;
    void AddPoints(const std::vector<Eigen::Vector4d> &points);
    Vector4dVectorTuple GetCorrespondences(
                    const V4d &points, double max_correspondance_distance);
    Sophus::SE3d RegisterFrameSemantic(const std::vector<Eigen::Vector4d> &frame,
                           const Sophus::SE3d &initial_guess,
                           double max_correspondence_distance);
public:
    FastIcp(float voxel_size_map,int max_points_per_voxel);
    ~FastIcp();
    Eigen::Matrix4d get_trans(const V3d_i &cloudA, const V3d_i &cloudB, const Eigen::Matrix4d &initTrans);

};


} // namespace graph_slam