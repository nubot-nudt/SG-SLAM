/* 
    SemGraphSLAM is heavily inspired by the framework of KISS-ICP (https://github.com/PRBonn/kiss-icp).
    We are deeply appreciative of Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss for their contributions to the open-source community.
*/

// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#include "Registration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>
#include <chrono>
#include <iostream>
#include <fstream>

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace {

inline double square(double x) { return x * x; }

struct ResultTuple {
    ResultTuple() {
        JTJ.setZero();
        JTr.setZero();
    }

    ResultTuple operator+(const ResultTuple &other) {
        this->JTJ += other.JTJ;
        this->JTr += other.JTr;
        return *this;
    }

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
};

void TransformPoints4D(const Sophus::SE3d &T, std::vector<Eigen::Vector4d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const Eigen::Vector4d &point) { 
                    Eigen::Vector4d pc_out;
                    pc_out.head<3>() = T * point.head<3>();
                    pc_out(3) = point(3);
                    return pc_out; });
}



std::pair<std::vector<Eigen::Vector3d>,std::vector<int>> SeparatePointsAndLabels(const std::vector<Eigen::Vector4d> &frame){
    std::vector<Eigen::Vector3d> pointcloud(frame.size());
    std::vector<int> label(frame.size());
    tbb::parallel_for(size_t(0),frame.size(), [&](size_t i){
        pointcloud[i] =  frame[i].head<3>();
        label[i] = (int)frame[i](3);
    });
    return std::make_pair(pointcloud,label);
}

Sophus::SE3d AlignClouds(const std::vector<Eigen::Vector4d> &source4d,
                         const std::vector<Eigen::Vector4d> &target4d,
                         double th) {
    
    const auto source_pl =  SeparatePointsAndLabels(source4d);
    const auto target_pl =  SeparatePointsAndLabels(target4d);

    const std::vector<Eigen::Vector3d> source = source_pl.first;
    const std::vector<Eigen::Vector3d> target = target_pl.first;
    const std::vector<int> label = source_pl.second;

    auto compute_jacobian_and_residual = [&](auto i) {
        const Eigen::Vector3d residual = source[i] - target[i];
        const int semantic_label = label[i];
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source[i]); 
        return std::make_tuple(J_r, residual,semantic_label);
    };

    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<size_t>{0, source.size()},
        // Identity
        ResultTuple(),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
            auto Weight = [&](double residual2) { return square(th) / square(th + residual2); }; // weigth from the residual distance
            auto &[JTJ_private, JTr_private] = J;
            for (auto i = r.begin(); i < r.end(); ++i) {
                const auto &[J_r, residual, semantic_label] = compute_jacobian_and_residual(i);
                const double w = Weight(residual.squaredNorm());
                JTJ_private.noalias() += J_r.transpose() * w * J_r;
                JTr_private.noalias() += J_r.transpose() * w * residual * weights_semantic[semantic_label] ; // label-specific weight
            }
            return J;
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        [&](ResultTuple a, const ResultTuple &b) -> ResultTuple { return a + b; }); 

    const Eigen::Vector6d x = JTJ.ldlt().solve(-JTr); 
    return Sophus::SE3d::exp(x);  
}

constexpr int MAX_NUM_ITERATIONS_ = 500;
constexpr double ESTIMATION_THRESHOLD_ = 0.0001;

}  // namespace

namespace graph_slam {


void writePointCloud(const std::vector<Eigen::Vector4d>& points, const std::string& filename){
    struct PointXYZL {
        float x;
        float y;
        float z;
        float label;
    };
    std::ofstream file(filename, std::ios::out | std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing\n";
        return;
    }
    std::vector<PointXYZL> converted_points(points.size());
    for(size_t i=0; i< points.size(); i++){
        converted_points[i].x =  static_cast<float>(points[i](0));
        converted_points[i].y =  static_cast<float>(points[i](1));
        converted_points[i].z =  static_cast<float>(points[i](2));
        converted_points[i].label =  static_cast<float>(points[i](3));
    };
    file.write(reinterpret_cast<char*>(converted_points.data()), sizeof(PointXYZL)*points.size());
    file.close();
}


Sophus::SE3d RegisterFrameSemantic(const std::vector<Eigen::Vector4d> &frame,
                           const VoxelHashMap &voxel_map,
                           const Sophus::SE3d &initial_guess,
                           double max_correspondence_distance,
                           double kernel) {
    if (voxel_map.Empty()) return initial_guess;

    // Transform the source points to the local frame using the initial guess
    std::vector<Eigen::Vector4d> source = frame;
    TransformPoints4D(initial_guess, source);
    // ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        // find correspondences: point-to-local pc map
        const auto &[src, tgt] = voxel_map.GetCorrespondences(source, max_correspondence_distance);
        // main computation
        auto estimation = AlignClouds(src, tgt, kernel);
        // Transform the source points to the global frame using the estimated transformation
        TransformPoints4D(estimation, source);
        // Update iterations
        T_icp = estimation * T_icp;
        // Termination criteria
        if (estimation.log().norm() < ESTIMATION_THRESHOLD_) 
        {
            break;
        }
    }
    
    // Spit the final transformation
    return T_icp * initial_guess;
}

}   // namespace graph_slam
