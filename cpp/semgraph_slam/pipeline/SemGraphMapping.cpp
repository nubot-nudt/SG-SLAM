// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#include "SemGraphMapping.hpp"


namespace graph_slam{


void SemGraphMapping::mainProcess(int cloud_id, const Graph &frame_graph){


    loop_flag = false; //flag for loop detection
    int num_exclude_curr = 150/config_.keyframe_interval; //the number of keyframes to be excluded from current frame

    time_gen_des = 0;
    time_search = 0;
    is_keyframe_vec.emplace_back(false); // store the keyframe flag for current frame
    
    std::vector<int> match_instance_idx;

    if(cloud_id%config_.keyframe_interval==0){
        
        is_keyframe_vec[cloud_id] = true;
        keyframe_graph_vec_.emplace_back(frame_graph);
        keyframe_idx_vec_.emplace_back(cloud_id);
        auto start_time_gen = std::chrono::steady_clock::now();

        // Generating scan descriptor
        const auto scan_descriptor = GenScanDescriptors(frame_graph,config_.edge_dis_th,config_.subinterval);
        scan_des_vec_.emplace_back(scan_descriptor);

        auto end_time_gen = std::chrono::steady_clock::now();
        
        auto start_time_search = std::chrono::steady_clock::now();

        // Searching loop closure
        if(int(keyframe_idx_vec_.size())>num_exclude_curr){
            loop_flag = SearchLoop(num_exclude_curr,match_instance_idx);
        }
        auto end_time_search = std::chrono::steady_clock::now();
        time_gen_des = std::chrono::duration<double, std::milli>(end_time_gen - start_time_gen).count();
        time_search = std::chrono::duration<double, std::milli>(end_time_search - start_time_search).count();
    }

    // Determine whether the instance node has been matched in the closed loop to prevent duplicate representation of the instance
    std::vector<bool> match_instance(frame_graph.node_labels.size(),false);  
    if(loop_flag){
        for(size_t i=0; i<match_instance_idx.size(); i++){
            match_instance[i] = true;
        }
    }
    std::vector<Eigen::Vector4d> new_instance_frame; 
    for(size_t i=0;i<frame_graph.new_instance.size();i++){
        int idx = frame_graph.new_instance[i];
        if(!match_instance[idx]) {
            new_instance_frame.emplace_back(Eigen::Vector4d(frame_graph.node_centers[idx][0],frame_graph.node_centers[idx][1],frame_graph.node_centers[idx][2],frame_graph.node_labels[idx]));
        }
    }

    // Record the number of instance nodes in the graph map 
    if(new_instance_frame_vec.size()==0) node_num_acc.emplace_back(0);
    else node_num_acc.emplace_back(node_num_acc.back()+new_instance_frame_vec[new_instance_frame_vec.size()-1].size());

    new_instance_frame_vec.emplace_back(new_instance_frame);
    
    int start_idx = std::max(0,int(new_instance_frame_vec.size()-30));

    // Rebuilding the edge information between current graph and all past graph
    for(size_t i=0; i<new_instance_frame.size();i++){

        for(size_t j=start_idx; j<(new_instance_frame_vec.size()-1); j++){

            for( size_t k=0; k<new_instance_frame_vec[j].size();k++){
                if((new_instance_frame[i].head<3>() - new_instance_frame_vec[j][k].head<3>()).norm()<config_.edge_dis_th)
                    global_graph_edge.emplace_back(std::make_pair(node_num_acc.back()+i,node_num_acc[j]+k));
            }
        }
    }

    // Rebuilding the edge information in current graph
    for(size_t i=0; i<new_instance_frame.size();i++){
        for(size_t j=i+1; j<new_instance_frame.size();j++){
            if((new_instance_frame[i].head<3>() - new_instance_frame[j].head<3>()).norm()<config_.edge_dis_th)
                global_graph_edge.emplace_back(std::make_pair(node_num_acc.back()+i,node_num_acc.back()+j));
        }
    }
}


/*
    Search loop closure and perform geometry verification to find the best loop closure pair in the past keyframes
*/
bool SemGraphMapping::SearchLoop(int num_exclude_curr, std::vector<int>& match_instance_idx){ 

    auto query_des = scan_des_vec_.back();
    auto idx_curre = keyframe_idx_vec_.back();
    auto graph_curre = keyframe_graph_vec_.back();

    std::vector<std::vector<float>> global_des_search;
    global_des_search.clear();
    global_des_search.assign( scan_des_vec_.begin(), scan_des_vec_.end() - num_exclude_curr ); // exclude the last num_exclude_curr keyframes

    // Building the global descriptor tree
    std::unique_ptr<InvDesTree> global_des_tree;
    global_des_tree = std::make_unique<InvDesTree>(config_.global_des_dim /* dim */, global_des_search, 10 /* max leaf */ );

    // Retrieving the nearest neighbor in the global descriptor tree
    std::vector<size_t> candidate_indexes(config_.search_results_num ); 
    std::vector<float> out_dists_sqr( config_.search_results_num );
    nanoflann::KNNResultSet<float> knnsearch_result( config_.search_results_num );
    knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
    global_des_tree->index->findNeighbors( knnsearch_result, &query_des[0] /* query */, nanoflann::SearchParameters(10) ); 

    bool search_results = false;
    Eigen::Matrix4d loop_trans = Eigen::Matrix4d::Identity();
    std::pair<int,int>loop_pair;

    // Loop closure verification
    for(int m = 0; m < config_.search_results_num; m++){
            if(out_dists_sqr[m]>config_.max_distance_for_loop) continue;
            int candi_idx_inkey = candidate_indexes[m]; // idx in keyfraem
            int candi_idx = keyframe_idx_vec_[candi_idx_inkey]; // idx in all frame
            
            search_results = GeometryVeriPoseEstimation(graph_curre,keyframe_graph_vec_[candi_idx_inkey],loop_trans,match_instance_idx,
                                                            config_.graph_sim_th, config_.back_sim_th, config_.map_voxel_size_loop);
            if(search_results){
                loop_pair.first = idx_curre;
                loop_pair.second = candi_idx;
                loop_pair_vec.emplace_back(loop_pair);
                loop_trans_vec.emplace_back(loop_trans);
                loop_frame_key_idx_vec.emplace_back(candi_idx_inkey); //for visualization
                break;
            }
    }
    return search_results;
}

/*
    graph map update
*/
void SemGraphMapping::UpdateMapping(const std::vector<Sophus::SE3d> &poses, int recent_update_idx){
    
    // latest poses
    global_graph_map.clear();
    for(int i=0; i<=recent_update_idx; i++){
        if(i>(int)new_instance_frame_vec.size()) break;
        const auto &new_instance_frame = new_instance_frame_vec[i];
        const auto &pose = poses[i]; // pose of current frame
        for(size_t j=0; j<new_instance_frame.size(); j++){
            int label = new_instance_frame[j][3];
            Eigen::Vector3d center = pose*new_instance_frame[j].head<3>();
            global_graph_map.emplace_back(Eigen::Vector4d(center[0],center[1],center[2],label));
        }
    }

}
} // namespace graph_slam