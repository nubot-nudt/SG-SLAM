// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <chrono>
#include <sophus/se3.hpp>

// SG-SLAM
#include "semgraph_slam/loopclosure/LoopClosure.hpp"
#include "semgraph_slam/core/Coreutils.h"

namespace graph_slam{

struct SemGraphMappingConfig{

    int global_des_dim = 231;   // the dimension of global descriptor
    int loop_candidate = 5;     // the number of loop candidates to be considered
    double edge_dis_th =  60;   // edge distance threshold for graph
    int subinterval = 30;       // subinterval for graph
    int keyframe_interval = 5;  //keyframe interval for graph
    int search_results_num = 5; //the number of search results to be considered
    float max_distance_for_loop =  0.07; //the maximum global descriptor distance for loop closure detection
    double graph_sim_th = 0.5; // graph similarity threshold for loop closure detection
    double back_sim_th = 0.58; // backgraoun similarity threshold for loop closure detection

    // map_voxel_size_loop should be 4*voxel_size_cluster in odometry
    double map_voxel_size_loop = 1.0; // the size of the (foreground and background) map voxel for loop closure ICP;
    int frame_acc_pgo  = 20;    // frame accumulation for pgo
    bool loop_closure_enable = true; // enable loop closure detection
};

class SemGraphMapping
{

private:
    SemGraphMappingConfig config_;
    using InvDesTree = KDTreeVectorOfVectorsAdaptor< std::vector<std::vector<float>>, float >;
public:
    explicit SemGraphMapping(const SemGraphMappingConfig& config)
        :config_(config){};

    SemGraphMapping(): SemGraphMapping(SemGraphMappingConfig()){}; // default constructor
    ~SemGraphMapping(){};

public:
    void mainProcess(int cloud_id, const Graph &frame_graph);
    void UpdateMapping(const std::vector<Sophus::SE3d> &pose, int recent_update_idx);

    bool loop_flag = false;
    std::vector<bool> is_keyframe_vec;      // the vector to indicate whether the frame is a keyframe
    
    std::vector<V4d> new_instance_frame_vec;            // the vector to store the new instance in each frame, for global graph construction
    std::vector<std::pair<int,int>> global_graph_edge;  // the vector to store the edges in the global graph
    std::vector<int> node_num_acc;                      // the vector to store the number of nodes in each frame
    std::vector<Eigen::Vector4d> global_graph_map;      // the vector to store the global graph map

    std::vector<std::pair<int,int>> loop_pair_vec;      // the vector to store the loop candidate pairs
    std::vector<Eigen::Matrix4d> loop_trans_vec;        // the vector to store the loop candidate transformations
    std::vector<int> loop_frame_key_idx_vec;            //for visualization
    double time_gen_des = 0;     // time for generating descriptors 
    double time_search = 0;      // time for searching loop candidates
    std::vector<int> keyframe_idx_vec_;             // the vector to store the keyframe indices in all frames
    std::vector<std::vector<float>> scan_des_vec_;  // the vector to store the scan descriptors in all keyframes
    std::vector<Graph> keyframe_graph_vec_;         // the vector to store the keyframe graphs

private:
    
    bool SearchLoop(int num_exclude_curr,std::vector<int>& match_instance_idx);
};

}