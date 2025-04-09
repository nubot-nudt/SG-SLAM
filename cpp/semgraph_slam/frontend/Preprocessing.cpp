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
#include "Preprocessing.hpp"

#include <tbb/parallel_for.h>
#include <tsl/robin_map.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <sophus/se3.hpp>
#include <vector>
#include <fstream>
#include <chrono>
#include <iostream>
#include <omp.h>


namespace {
using Voxel = Eigen::Vector3i;
struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};
}  // namespace

namespace graph_slam {
std::vector<Eigen::Vector3d> VoxelDownsample(const std::vector<Eigen::Vector3d> &frame,
                                             double voxel_size) {

    tsl::robin_map<Voxel, Eigen::Vector3d, VoxelHash> grid;
    grid.reserve(frame.size());
    for (const auto &point : frame) {
        const auto voxel = Voxel((point / voxel_size).cast<int>());
        if (grid.contains(voxel)) continue;
        grid.insert({voxel, point});
    }

    std::vector<Eigen::Vector3d> frame_dowsampled;
    frame_dowsampled.reserve(grid.size());
    for (const auto &[voxel, point] : grid) {
        (void)voxel;
        frame_dowsampled.emplace_back(point);
    }
    return frame_dowsampled;
}

V3d_i VoxelDownsampleSemantic(const V3d_i &frame,
                                             double voxel_size) {
    //TODO: Adaptive voxel downsampling
    tsl::robin_map<Voxel, int, VoxelHash> grid;
    grid.reserve(frame.first.size());
 
    for(int i=0;i<(int)frame.first.size();i++){
                const auto voxel = Voxel((frame.first[i] / voxel_size).cast<int>());
                if (grid.contains(voxel) ) continue;
                grid.insert({voxel, i});
    }

    std::vector<Eigen::Vector3d> frame_downsampled;
    std::vector<int> frame_label_downsampled;

    frame_downsampled.reserve(grid.size());
    frame_label_downsampled.reserve(grid.size());
    for (const auto &[voxel, index] : grid) {
        (void)voxel;
        frame_downsampled.emplace_back(frame.first[index]);
        frame_label_downsampled.emplace_back(frame.second[index]);
    }

    return std::make_pair(frame_downsampled,frame_label_downsampled);
}

std::vector<Eigen::Vector3d> Preprocess(const std::vector<Eigen::Vector3d> &frame,
                                        double max_range,
                                        double min_range) {
    std::vector<Eigen::Vector3d> inliers;
    std::copy_if(frame.cbegin(), frame.cend(), std::back_inserter(inliers), [&](const auto &pt) {
        const double norm = pt.norm();
        return norm < max_range && norm > min_range;
    });
    return inliers;
}

V3d_i PreprocessSemantic(const std::vector<Eigen::Vector3d> &frame,
                                        const std::vector<int> &label,
                                        double max_range,
                                        double min_range) {
    assert(frame.size() == label.size());
    std::vector<Eigen::Vector3d> inliers;
    std::vector<int> inliers_label;
    for(int i=0; i<(int)frame.size();i++)
    {
        const double norm = frame[i].norm();
        if(norm<max_range && norm>min_range && label[i]<20)  // removing the dymaic objects, label value < 20 from the segnet4d 
        {
            inliers.emplace_back(frame[i]);
            inliers_label.emplace_back(label[i]);
        }
    }

    return std::make_pair(inliers,inliers_label);
}

std::vector<Eigen::Vector3d> CorrectKITTIScan(const std::vector<Eigen::Vector3d> &frame) {
    constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;
    std::vector<Eigen::Vector3d> corrected_frame(frame.size());
    tbb::parallel_for(size_t(0), frame.size(), [&](size_t i) {
        const auto &pt = frame[i];
        const Eigen::Vector3d rotationVector = pt.cross(Eigen::Vector3d(0., 0., 1.));
        corrected_frame[i] =
            Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET, rotationVector.normalized()) * pt;
    });
    return corrected_frame;
}
}   // namespace graph_slam
