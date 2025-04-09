// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#include <tbb/parallel_for.h>

#include "SemGraphSLAM.hpp"

#include "semgraph_slam/frontend/Deskew.hpp"
#include "semgraph_slam/frontend/Preprocessing.hpp"
#include "semgraph_slam/frontend/Registration.hpp"
#include "semgraph_slam/frontend/VoxelHashMap.hpp"
#include "semgraph_slam/loopclosure/LoopClosure.hpp"

namespace graph_slam{

SemGraphSLAM::V3d_i_pair_graph SemGraphSLAM::mainProcess(const V3d &frame, const std::vector<int> &frame_label, const std::vector<double> &timestamps,std::string dataset){

    V3d deskew_frame = frame;

    // Deskew
    if(config_.deskew&&!timestamps.empty()){
        const size_t N = poses().size();
        if(N>2){
            const auto &start_pose = poses_[N - 2];
            const auto &finish_pose = poses_[N - 1];
            deskew_frame = DeSkewScan(frame, timestamps, start_pose, finish_pose);
        }
    }

    if(dataset=="kitti"){
        // Correct KITTI scan
        deskew_frame = CorrectKITTIScan(frame);
    }
        
    // Preprocess frame
    const auto &cropped_frame_label = PreprocessSemantic(deskew_frame,frame_label, config_.max_range, config_.min_range);

    // Voxel downsample
    const auto &[source, frame_downsample, frame_downsample_cluster] = VoxelizeSemantic(cropped_frame_label);

    // Cluster
    V3d_i foreground_points;
    V3d_i background_points;
    auto cluster_box = ClusterPoints(frame_downsample_cluster.first, frame_downsample_cluster.second,
                                     background_points, foreground_points,
                                     config_.deltaA, config_.deltaR, config_.deltaP);

    // Build frame graph
    auto graph = BuildGraph(cluster_box,config_.edge_dis_th,config_.subinterval,config_.graph_node_dimension,config_.subgraph_edge_th);
    graph.back_points = background_points;
    graph.front_points = foreground_points;

    // Node tracking: find instance node match
    const auto frame2map_match =  local_graph_map_.FindInsMatch(graph);
    
    
    // Get motion prediction and adaptive_threshold
    const double sigma = GetAdaptiveThreshold();

    // Compute initial_guess for ICP
    const auto prediction = GetPredictionModel();
    const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess = last_pose * prediction;

    initial_guess_for_relocalization = initial_guess;

    // Fuse (vector3d point, label) -> vector 4d point4 for subsequent processing
    const auto source_4d = FusePointsAndLabels(source);
    const auto frame_downsample_4d = FusePointsAndLabels(frame_downsample);

    // Registration
    Sophus::SE3d new_pose = RegisterFrameSemantic(source_4d,         // the current point cloud
                                                          local_map_,     // the local pc map
                                                          initial_guess,  // initial guess
                                                          3.0 * sigma,    // max_correspondence_distance
                                                          sigma / 3.0);   // kernel

    // The deviation between the initial guess and the final pose
    auto model_deviation = initial_guess.inverse() * new_pose;

    // Relocalization
    relocalization_corr = std::make_pair(V3d(),V3d());
    if(poses_.size()>2 && config_.relocalization_enable){ // relocalization enable

        // Check model deviation
        if(model_deviation.translation().norm()>config_.model_deviation_trans || model_deviation.so3().log().norm()>config_.model_deviation_rot){

            std::cout<<YELLOW<<"[ Relo. ] relocalization"<<std::endl;
            const auto [initial_guess_graph, estimate_poses_flag] = local_graph_map_.Relocalization(graph, frame2map_match,config_.inlier_rate_th);

            std::cout<<YELLOW<<"[ Relo. ] estimate_poses_flag:"<<estimate_poses_flag<<std::endl;
            if(estimate_poses_flag){ // successful relocalization
                // Regisration with relocalized poses
                new_pose = RegisterFrameSemantic(source_4d,         //
                                                    local_map_,     //
                                                    initial_guess_graph,  // the relocalized poses
                                                    3.0 * sigma,    //
                                                    sigma / 3.0);
                model_deviation = Sophus::SE3d();
                relocalization_corr = local_graph_map_.relo_corr;
            }
            else{
                relocalization_corr = std::make_pair(V3d(),V3d());
            }
        }

    }
     
    // Update constant motion model
    adaptive_threshold_.UpdateModelDeviation(model_deviation);

    // Update local point cloud map
    local_map_.Update(frame_downsample_4d, new_pose);

    // Update local graph map
    local_graph_map_.Update(graph, frame2map_match, new_pose);

    // Push new pose to global poses
    poses_.push_back(new_pose);
    return {frame_downsample_cluster, source, graph};
}

// Voxel downsample the semantic frame
SemGraphSLAM::V3d_i_tuple SemGraphSLAM::VoxelizeSemantic(const V3d_i &frame) const {
    const auto voxel_size = config_.voxel_size;
    const auto voxel_size_cluster = config_.voxel_size_cluster;

    const auto frame_downsample_cluster = VoxelDownsampleSemantic(frame, voxel_size_cluster);
    const auto frame_downsample = VoxelDownsampleSemantic(frame, voxel_size * 0.5);
    const auto source = VoxelDownsampleSemantic(frame_downsample, voxel_size * 1.5);
    return {source, frame_downsample,frame_downsample_cluster};
}

//Fusing the points and labels, time comsumption: ~0.02ms
V4d  SemGraphSLAM::FusePointsAndLabels(const V3d_i &frame){
    assert(frame.first.size()==frame.second.size());
    V4d points(frame.first.size());

    tbb::parallel_for(size_t(0),frame.first.size(), [&](size_t i){
        points[i].head<3>() =  frame.first[i];
        points[i](3) = frame.second[i];
    });
    return points;
}


double SemGraphSLAM::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_threshold;
    }
    return adaptive_threshold_.ComputeThreshold();
    // return config_.initial_threshold;
}

bool SemGraphSLAM::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}

Sophus::SE3d SemGraphSLAM::GetPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}

}
