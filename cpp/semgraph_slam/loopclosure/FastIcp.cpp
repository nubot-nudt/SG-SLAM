// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>


#include <iostream>
#include "FastIcp.hpp"



namespace graph_slam{

FastIcp::FastIcp(float voxel_size_map,int max_points_per_voxel)
{
    map_voel_size_ = voxel_size_map;
    max_points_per_voxel_ = max_points_per_voxel;
}

FastIcp::~FastIcp()
{
}

std::vector<Eigen::Vector4d>  FastIcp::FusePointsAndLabels(const V3d_i &frame){
    assert(frame.first.size()==frame.second.size());
    std::vector<Eigen::Vector4d> points(frame.first.size());

    tbb::parallel_for(size_t(0),frame.first.size(), [&](size_t i){
        points[i].head<3>() =  frame.first[i];
        points[i](3) = frame.second[i];
    });
    return points;
}

void FastIcp::AddPoints(const std::vector<Eigen::Vector4d> &points) {
    std::for_each(points.cbegin(), points.cend(), [&](const Eigen::Vector4d &point) {
        auto voxel = Eigen::Vector3i((point.head<3>()/map_voel_size_).template cast<int>());
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto &voxel_block = search.value();
            voxel_block.AddPoint(point);
        } else {
            map_.insert({voxel, VoxelBlock{{point}, max_points_per_voxel_}});
        }
    });
}

Vector4dVectorTuple FastIcp::GetCorrespondences(
    const V4d &points, double max_correspondance_distance)  {
    auto GetClosestNeighboor = [&](const Eigen::Vector4d &point) {
        auto kx = static_cast<int>(point[0] / map_voel_size_);
        auto ky = static_cast<int>(point[1] / map_voel_size_);
        auto kz = static_cast<int>(point[2] / map_voel_size_);
        std::vector<Eigen::Vector3i> voxels;
        voxels.reserve(27);
        for (int i = kx - 1; i < kx + 1 + 1; ++i) {
            for (int j = ky - 1; j < ky + 1 + 1; ++j) {
                for (int k = kz - 1; k < kz + 1 + 1; ++k) {
                    voxels.emplace_back(i, j, k);
                }
            }
        }

        V4d neighboors;
        neighboors.reserve(27 * max_points_per_voxel_);
        std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
            auto search = map_.find(voxel);
            if (search != map_.end()) {
                const auto &points = search->second.points;
                if (!points.empty()) {
                    for (const Eigen::Vector4d &point : points) {
                        neighboors.emplace_back(point);
                    }
                }
            }
        });
        Eigen::Vector4d closest_neighbor;
        double closest_distance2 = std::numeric_limits<double>::max();
        std::for_each(neighboors.cbegin(), neighboors.cend(), [&](const Eigen::Vector4d &neighbor) {
            double distance = (neighbor.head<3>() - point.head<3>()).squaredNorm();
            if (distance < closest_distance2) { 
                closest_neighbor = neighbor;
                closest_distance2 = distance;
            }
        });

        return closest_neighbor;
    };
    

    using points_iterator = std::vector<Eigen::Vector4d>::const_iterator;
    const auto [source, target] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
        
        // Identity
        ResultTuple(points.size()),
        // 1st lambda: Parallel computation
        [max_correspondance_distance, &GetClosestNeighboor](
            const tbb::blocked_range<points_iterator> &r, ResultTuple res) -> ResultTuple {
            auto &[src, tgt] = res;
            src.reserve(r.size());
            tgt.reserve(r.size());
            for (const Eigen::Vector4d &point : r) {
                Eigen::Vector4d closest_neighboors = GetClosestNeighboor(point);
                if ((closest_neighboors.head<3>() - point.head<3>()).norm() < max_correspondance_distance) {
                    src.emplace_back(point);
                    tgt.emplace_back(closest_neighboors);
                }
            }
            return res;
        },
        // 2nd lambda: Parallel reduction
        [](ResultTuple a, const ResultTuple &b) -> ResultTuple {
            auto &[src, tgt] = a;
            const auto &[srcp, tgtp] = b;
            src.insert(src.end(),  //
                       std::make_move_iterator(srcp.begin()), std::make_move_iterator(srcp.end()));
            tgt.insert(tgt.end(),  //
                       std::make_move_iterator(tgtp.begin()), std::make_move_iterator(tgtp.end()));
            return a;
        });

    return std::make_tuple(source, target);
}

V3d_i FastIcp::SeparatePointsAndLabels(const std::vector<Eigen::Vector4d> &frame){
    std::vector<Eigen::Vector3d> pointcloud(frame.size());
    std::vector<int> label(frame.size());
    tbb::parallel_for(size_t(0),frame.size(), [&](size_t i){
        pointcloud[i] =  frame[i].head<3>();
        label[i] = (int)frame[i](3);
    });
    return std::make_pair(pointcloud,label);
}

inline double square(double x) { return x * x; }

Sophus::SE3d FastIcp::AlignClouds(const std::vector<Eigen::Vector4d> &source4d,
                         const std::vector<Eigen::Vector4d> &target4d) {
    
    const auto source_pl =  SeparatePointsAndLabels(source4d);
    const auto target_pl =  SeparatePointsAndLabels(target4d);

    const std::vector<Eigen::Vector3d> source = source_pl.first;
    const std::vector<Eigen::Vector3d> target = target_pl.first;

    auto compute_jacobian_and_residual = [&](auto i) {
        const Eigen::Vector3d residual = source[i] - target[i];
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat(source[i]); 
        return std::make_tuple(J_r, residual);
    };

    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        tbb::blocked_range<size_t>{0, source.size()},
        ResultTupleRe(),
        [&](const tbb::blocked_range<size_t> &r, ResultTupleRe J) -> ResultTupleRe {
            auto &[JTJ_private, JTr_private] = J;
            for (auto i = r.begin(); i < r.end(); ++i) {
                const auto &[J_r, residual] = compute_jacobian_and_residual(i);
                JTJ_private.noalias() += J_r.transpose()  * J_r;
                JTr_private.noalias() += J_r.transpose()  * residual;
            }
            return J;
        },
        [&](ResultTupleRe a, const ResultTupleRe &b) -> ResultTupleRe { return a + b; });

    const Eigen::Vector6d x = JTJ.ldlt().solve(-JTr); 
    return Sophus::SE3d::exp(x);  
}

void FastIcp::TransformPoints4D(const Sophus::SE3d &T, std::vector<Eigen::Vector4d> &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const Eigen::Vector4d &point) { 
                    Eigen::Vector4d pc_out;
                    pc_out.head<3>() = T * point.head<3>();
                    pc_out(3) = point(3);
                    return pc_out; });
}

Sophus::SE3d FastIcp::RegisterFrameSemantic(const std::vector<Eigen::Vector4d> &frame,
                           const Sophus::SE3d &initial_guess,
                           double max_correspondence_distance) {
    if (map_.empty()) return initial_guess;

    std::vector<Eigen::Vector4d> source = frame;
    TransformPoints4D(initial_guess, source);
    Sophus::SE3d T_icp = Sophus::SE3d();
    for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        const auto &[src, tgt] = GetCorrespondences(source, max_correspondence_distance);
        auto estimation = AlignClouds(src, tgt);

        TransformPoints4D(estimation, source);
        T_icp = estimation * T_icp;
        if (estimation.log().norm() < ESTIMATION_THRESHOLD_) 
        {
            break;
        }
    }


    return T_icp * initial_guess;
}

Eigen::Matrix4d FastIcp::get_trans(const V3d_i &cloudA, const V3d_i &cloudB, const Eigen::Matrix4d &initTrans){

    //Fuse vector3d point, label -> vector 4d point4
    const auto source_a_4d = FusePointsAndLabels(cloudA);
    const auto source_b_4d = FusePointsAndLabels(cloudB);

    // 
    map_.clear();
    AddPoints(source_b_4d);
    Sophus::SE3d se3_init(initTrans.block<3, 3>(0, 0), initTrans.block<3, 1>(0, 3));
    const Sophus::SE3d new_pose = RegisterFrameSemantic(source_a_4d, se3_init, 0.5);
    Eigen::Matrix4d new_pose_m4d = Eigen::Matrix4d::Identity();
    new_pose_m4d.block<3, 3>(0, 0) = new_pose.rotationMatrix();
    new_pose_m4d.block<3, 1>(0, 3) = new_pose.translation();
    return new_pose_m4d;
}


}