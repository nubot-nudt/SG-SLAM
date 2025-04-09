// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>


#include "GraphMap.hpp"



namespace graph_slam{

bool CheckSubTriangle(const std::vector<Eigen::Vector3d> &nodeSubgraphTriangle1, const std::vector<Eigen::Vector3d> &nodeSubgraphTriangle2){

    int matche_sub_Triangle_th = 0;
    bool sub_triangle_match = false;
    if(nodeSubgraphTriangle1.size()<2 && nodeSubgraphTriangle2.size()<2) return true; // 节点没有子节点三角形，直接返回true,无法判断
    // 匹配两个对应节点的局部子图三角形
    for(size_t m=0;m<nodeSubgraphTriangle1.size();m++){
        for(size_t n=0;n<nodeSubgraphTriangle2.size();n++){
            if(abs(nodeSubgraphTriangle1[m].x() - nodeSubgraphTriangle2[n].x())<0.3&&
                abs(nodeSubgraphTriangle1[m].y() - nodeSubgraphTriangle2[n].y())<0.3&&
                abs(nodeSubgraphTriangle1[m].z() - nodeSubgraphTriangle2[n].z())<0.3){
                    matche_sub_Triangle_th++;
            }
        }
    }
    // if(matche_sub_Triangle_th>=1) {
    //         sub_triangle_match = true;
    //     }
    if(nodeSubgraphTriangle1.size()>5 && nodeSubgraphTriangle2.size()>5){
        if(matche_sub_Triangle_th>=3) { 
            sub_triangle_match = true;
        }
    }
    else{
        if(matche_sub_Triangle_th>=1) {
            sub_triangle_match = true;
        }
    }
    return sub_triangle_match;
}


std::tuple<V3d_i,V3d_i> FindCorrespondencesKDtree(const Graph &graph1, const Graph &graph2, int search_results_num){// graph1是frame, graph2是map

    using NodeDesTree = KDTreeVectorOfVectorsAdaptor< std::vector<std::vector<float>>, float >;
    
    std::vector<std::vector<float>> map_node_des_vec;
    for(auto node_desc:graph2.node_desc){
        map_node_des_vec.emplace_back(node_desc);
    }
    std::unique_ptr<NodeDesTree> map_node_des_tree;
    int dim = graph2.node_desc[0].size();
    map_node_des_tree = std::make_unique<NodeDesTree>(dim /* dim */, map_node_des_vec, 10 /* max leaf */ );

    std::vector<Eigen::Vector3d> query_nodes_center;
    std::vector<Eigen::Vector3d> match_nodes_center;
    std::vector<int> query_nodes_idx;
    std::vector<int> match_nodes_idx;

    for(size_t i=0;i<graph1.node_desc.size();i++){

        auto query_des = graph1.node_desc[i];
        std::vector<size_t> candidate_indexes(search_results_num ); 
        std::vector<float> out_dists_sqr(search_results_num );

        nanoflann::KNNResultSet<float> knnsearch_result(search_results_num );
        knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
        map_node_des_tree->index->findNeighbors( knnsearch_result, &query_des[0] /* query */, nanoflann::SearchParameters(10));

        for(int m=0;m<search_results_num;m++){
            int j = candidate_indexes[m];
            if((graph1.node_labels[i] == graph2.node_labels[j]) && CheckSubTriangle(graph1.node_sub_triangles[i],graph2.node_sub_triangles[j])){
                query_nodes_center.emplace_back(graph1.node_centers[i]);
                query_nodes_idx.emplace_back(i);
                match_nodes_center.emplace_back(graph2.node_centers[j]);
                match_nodes_idx.emplace_back(j);
                break;
            }
        }
    }
    return {std::make_pair(query_nodes_center,query_nodes_idx),std::make_pair(match_nodes_center,match_nodes_idx)};
}

// VTbii = std::vector<std::tuple<bool,int,int>>;  // (match flag, instance id in current graph, instnace in local map)
/*
    Find instance correspondences between current graph and local graph map
*/
VTbii GraphMap::FindInsMatch(const Graph &graph) {

    VTbii frame2map_match;
    Sophus::SE3d poses_init = Sophus::SE3d();
    if(InsGraphMap.node_labels.empty()) { // add first frame to localmap
        for(size_t i=0; i<graph.node_labels.size(); i++){
            frame2map_match.emplace_back(false, i, -1);
        }
        return frame2map_match;
    }

    // find node correspondences between current graph and local graph map
    auto [match_node_inGraph_q,match_node_inGraph_c] = FindCorrespondencesKDtree(graph,InsGraphMap,5);
    std::vector<bool> is_match(graph.node_labels.size(),false);
    
    // build macth index
    for(size_t i=0; i<match_node_inGraph_q.second.size(); i++){
            frame2map_match.emplace_back(true, match_node_inGraph_q.second[i], match_node_inGraph_c.second[i]);
            is_match[match_node_inGraph_q.second[i]] = true;
    }
    for(size_t i=0; i<is_match.size(); i++)
    {
        if(!is_match[i])  frame2map_match.emplace_back(false, i, -1);
    }
    return frame2map_match;
}

/*
    Relocalization when odometry fails
*/
std::tuple< Sophus::SE3d, bool> GraphMap::Relocalization(const Graph &graph,const VTbii &frame2map_match,double inlier_rate_th){

    bool estimate_poses_flag = false;
    Sophus::SE3d poses_init = Sophus::SE3d();
    std::vector<Eigen::Vector3d> match_node1;
    std::vector<Eigen::Vector3d> match_node2;
    for(auto [match_flag, id_in_frame, id_in_localmap] : frame2map_match){
        if(match_flag){
            assert(id_in_localmap>0);
            match_node1.emplace_back(graph.node_centers[id_in_frame]);
            match_node2.emplace_back(InsGraphMap.node_centers[id_in_localmap]);
        }
    }
    relo_corr = std::make_pair(match_node1,match_node2); // for visualization
    std::cout<<YELLOW<<"[ Debug ] match_node_inGraph_q size:"<<match_node1.size()<<std::endl;
    if(match_node1.size()>3){
        int best_inlier = 0;

        auto [trans,score] = RansacAlignment(match_node1,match_node2,graph,InsGraphMap,500,best_inlier);
        
        if(score>inlier_rate_th)  estimate_poses_flag = true; 
        std::cout<<YELLOW<<"[ Debug ] score:"<<score<<std::endl;
        poses_init = Sophus::SE3d(trans.matrix());
    }
    
    return std::make_tuple(poses_init, estimate_poses_flag);
}

/*
    Graph map update
*/
void GraphMap::Update(Graph &graph, const VTbii &frame2map_match, const Sophus::SE3d &pose){

    // create instance map hash table for fast search
    tsl::robin_map<Eigen::Vector3i, IndexBlock, VoxelHash> instance_map_hash;
    for(size_t idx=0; idx<instance_in_localmap.size(); idx++){
        auto voxel = Eigen::Vector3i((instance_in_localmap[idx].pose.head<3>()/voxel_size_).template cast<int>());
        auto search = instance_map_hash.find(voxel);
        if (search != instance_map_hash.end()) {
            auto &index_block = search.value();
            index_block.AddIndex(idx);
        } else {
            instance_map_hash.insert({voxel, IndexBlock{{int(idx)}}});
        }
    }

    for(auto &[match_flag, id_in_frame, id_in_localmap] : frame2map_match){
        
        bool f2m_match_flag = match_flag;
        double neighboors_idstance_th;
        neighboors_idstance_th = graph.node_labels[id_in_frame]==1? nearest_neighbor_vehicle_disth_ : nearest_neighbor_pole_disth_;
        if(f2m_match_flag){ // if instance is already in localmap, 
            Eigen::Vector3d pose_ins = pose * graph.node_centers[id_in_frame];
            // judge if the new instance is close to the existing instance in localmap
            if((pose_ins-instance_in_localmap[id_in_localmap].pose).norm()<neighboors_idstance_th){ 
                instance_in_localmap[id_in_localmap].pose = (instance_in_localmap[id_in_localmap].pose + pose_ins)/2;
                instance_in_localmap[id_in_localmap].dimension = (instance_in_localmap[id_in_localmap].dimension  + graph.node_dimensions[id_in_frame])/2;
            }
            else f2m_match_flag = false;
        }
        
        // if instance is not in localmap, add it to localmap
        if(!f2m_match_flag){ 
            InsNode instance;
            double neighboors_idstance_th = nearest_neighbor_pole_disth_;

            // new instance point num and box diemension check
            if(graph.node_labels[id_in_frame]==1){
                if(graph.points_num[id_in_frame]<50) continue;
                if(graph.node_dimensions[id_in_frame][0]<1 || graph.node_dimensions[id_in_frame][1]<1 || graph.node_dimensions[id_in_frame][2]<1)  continue; 
            }
            else if(graph.node_labels[id_in_frame]==2 || graph.node_labels[id_in_frame]==3){
                if(graph.node_dimensions[id_in_frame][2]<1) continue;
            }
            instance.pose = pose * graph.node_centers[id_in_frame];
            instance.label = graph.node_labels[id_in_frame];
            instance.dimension = graph.node_dimensions[id_in_frame];
            instance.points_num = graph.points_num[id_in_frame];
            instance.id = instance_id_++;

            // determine if there are nearby instances in the local map, prevent duplicate additions
            if(IsExistNearestNeighbor(instance_map_hash,instance.pose,instance.label,neighboors_idstance_th)) continue;

            // new instance
            graph.new_instance.emplace_back(id_in_frame); 

            instance_in_localmap.emplace_back(instance);
        }
    }


    // delete out of range instance
    const Eigen::Vector3d &origin = pose.translation();
    const auto max_distance2 = max_distance_ * max_distance_;
    for (auto it = instance_in_localmap.begin(); it != instance_in_localmap.end(); ) {
        if ((it->pose-origin).squaredNorm() > max_distance2) {
            it = instance_in_localmap.erase(it); // allocate memory 
        } else {
            ++it; 
        }
    }

    // rebuild graph for next round matching: time consumption is very short, 0.1~1ms
    InsGraphMap = ReBuildGraph(instance_in_localmap,edge_dis_th_,subinterval_,graph_node_dimension_,subgraph_edge_th_);
}

/*
    Instance node NMS in local graph map
*/
bool GraphMap::IsExistNearestNeighbor(const tsl::robin_map<Eigen::Vector3i, IndexBlock, VoxelHash>& instance_poses_map,
                                        const Eigen::Vector3d & poses_ins, int labels, double neighbor_th){
    auto kx = static_cast<int>(poses_ins[0] / voxel_size_);
    auto ky = static_cast<int>(poses_ins[1] / voxel_size_);
    auto kz = static_cast<int>(poses_ins[2] / voxel_size_);
    std::vector<Eigen::Vector3i> voxels;

    voxels.reserve(27);
    for (int i = kx - 1; i < kx + 1 + 1; ++i) {
        for (int j = ky - 1; j < ky + 1 + 1; ++j) {
            for (int k = kz - 1; k < kz + 1 + 1; ++k) {
                voxels.emplace_back(i, j, k);
            }
        }
    }
    std::vector<int> neighbor_idx;
    std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
            auto search = instance_poses_map.find(voxel);
            if (search != instance_poses_map.end()) {
                const auto &index_v = search->second.index_instance;
                if (!index_v.empty()) {
                    for (const int &index : index_v) {
                        neighbor_idx.emplace_back(index);
                    }
                }
            }
    });
    
    for(const int &index : neighbor_idx){
        if(index<0 || index>=int(instance_in_localmap.size())) continue;
        if(instance_in_localmap[index].label!=labels) continue;
        if((poses_ins-instance_in_localmap[index].pose).norm()<neighbor_th) return true;
    }

    return false;
}

}