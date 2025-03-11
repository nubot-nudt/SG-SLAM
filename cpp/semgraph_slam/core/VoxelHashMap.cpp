// MIT License for KISS-ICP
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "VoxelHashMap.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>

#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>
#include <iostream>

// This parameters are not intended to be changed, therefore we do not expose it
namespace {
struct ResultTuple {
    ResultTuple(std::size_t n) {
        source.reserve(n);
        target.reserve(n);
    }
    std::vector<Eigen::Vector4d> source;
    std::vector<Eigen::Vector4d> target;
};
}  // namespace

namespace graph_slam {

VoxelHashMap::Vector4dVectorTuple VoxelHashMap::GetCorrespondences(
    const V4d &points, double max_correspondance_distance) const {
    // Lambda Function to obtain the KNN of one point, maybe refactor
    auto GetClosestNeighboor = [&](const Eigen::Vector4d &point) {
        auto kx = static_cast<int>(point[0] / voxel_size_);
        auto ky = static_cast<int>(point[1] / voxel_size_);
        auto kz = static_cast<int>(point[2] / voxel_size_);
        std::vector<Voxel> voxels;
        
        voxels.reserve(27);
        for (int i = kx - 1; i < kx + 1 + 1; ++i) {
            for (int j = ky - 1; j < ky + 1 + 1; ++j) {
                for (int k = kz - 1; k < kz + 1 + 1; ++k) {
                    voxels.emplace_back(i, j, k);
                }
            }
        }

        // using Vector4dVector = std::vector<Eigen::Vector4d>;
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
    

    //tbb并行
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

std::vector<Eigen::Vector4d> VoxelHashMap::Pointcloud() const {
    std::vector<Eigen::Vector4d> points;
    points.reserve(max_points_per_voxel_ * map_.size());
    for (const auto &[voxel, voxel_block] : map_) {
        (void)voxel;
        for (const Eigen::Vector4d &point : voxel_block.points) {
            // points.push_back(point.head<3>()); //only return x,y,z
            points.push_back(point);
        }
    }
    return points;
}

void VoxelHashMap::Update(const V4d &points, const Eigen::Vector3d &origin) {
    AddPoints(points);
    RemovePointsFarFromLocation(origin);
}

void VoxelHashMap::Update(const V4d &points, const Sophus::SE3d &pose) {
    V4d points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                   [&](const Eigen::Vector4d &point) { 
                        Eigen::Vector4d point_trans;
                        point_trans.head<3>() = pose * point.head<3>();
                        point_trans(3)  = point(3);
                        return point_trans;});
    const Eigen::Vector3d &origin = pose.translation();
    
    Update(points_transformed, origin);
}

void VoxelHashMap::AddPoints(const V4d &points) {
    // int max_points_per_voxel_pole_like = 20;
    std::for_each(points.cbegin(), points.cend(), [&](const Eigen::Vector4d &point) {
        auto voxel = Voxel((point.head<3>()/voxel_size_).template cast<int>());
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto &voxel_block = search.value();
            voxel_block.AddPoint(point);
        } else {
            map_.insert({voxel, VoxelBlock{{point}, max_points_per_voxel_}});
        }
    });
}

void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    for (const auto &[voxel, voxel_block] : map_) {
        const auto &pt = voxel_block.points.front().head<3>();
        const auto max_distance2 = max_distance_ * max_distance_;
        if ((pt - origin).squaredNorm() > (max_distance2)) {
            map_.erase(voxel);
        }
    }
}
}   // namespace graph_slam
