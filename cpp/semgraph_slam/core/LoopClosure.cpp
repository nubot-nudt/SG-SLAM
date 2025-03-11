// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>


#include "LoopClosure.hpp"



namespace graph_slam{


/*
    Generating scan descriptors = graph descriptors + back ground descriptors
*/
std::vector<float> GenScanDescriptors(const Graph &graph,double edge_dis_th,int subinterval){

    auto graph_des = GenGraphDescriptors(graph,edge_dis_th,subinterval);

    auto back_ground = GenBackDescriptors(graph.back_points);

    std::vector<float> global_graph_desp;
    global_graph_desp.insert(global_graph_desp.end(),graph_des.begin(),graph_des.end());
    global_graph_desp.insert(global_graph_desp.end(),back_ground.begin(),back_ground.end());

    float sum_of_squares_graph = std::inner_product(global_graph_desp.begin(), global_graph_desp.end(), global_graph_desp.begin(), 0.0f);
    float l2_norm_graph = std::sqrt(sum_of_squares_graph);
    if(l2_norm_graph==0) l2_norm_graph=0.000001f;
    for (auto it = global_graph_desp.begin(); it != global_graph_desp.end(); ++it) {
        *it /= l2_norm_graph; 
    }
    return global_graph_desp;
}

/*
    Geometry verification and pose estimation
*/
bool GeometryVeriPoseEstimation(const Graph &graph_q, const Graph &graph_t, Eigen::Matrix4d &transform, std::vector<int>& match_instance_idx,
                                double graph_sim_th,double back_sim_th,double map_voxel_size_loop){
    
    if(graph_q.node_centers.size()<3 || graph_t.node_centers.size()<3) return false;


    auto [match_node_inGraph_q,match_node_inGraph_c] = FindCorrespondencesWithIdx(graph_q,graph_t);
    auto [refine_match_node_inGraph_q,refine_match_node_inGraph_c] = OutlierPruning(graph_q,graph_t,match_node_inGraph_q,match_node_inGraph_c);

    if(refine_match_node_inGraph_q.first.size()>=3){

        // ransac alignment
        int best_inlier = 0;
        auto [trans,score] = RansacAlignment(refine_match_node_inGraph_q.first,refine_match_node_inGraph_c.first,match_node_inGraph_q.first,match_node_inGraph_c.first,200,best_inlier);

        auto R_coarse = trans.rotation();
        auto T_coarse = trans.translation();
        V3d_i cloud_back_q_trans = graph_q.back_points;
        std::transform(cloud_back_q_trans.first.cbegin(), cloud_back_q_trans.first.cend(), cloud_back_q_trans.first.begin(),
                [&](const Eigen::Vector3d &point) { 
                Eigen::Vector3d pc_out;
                pc_out = R_coarse * point + T_coarse;
                return pc_out; });

        auto desc1 = GenBackDescriptorsVeri(cloud_back_q_trans);
        auto desc2 = GenBackDescriptorsVeri(graph_t.back_points);
        double score_ssc = CalcaulateSim(desc1, desc2);
        Eigen::Vector3d translation = trans.matrix().block<3, 1>(0, 3);
        double distance = translation.norm();
        if(score<graph_sim_th || score_ssc<back_sim_th || distance>15) return false;
        
        // loop transformation refinement
        FastIcp instanceIcp(map_voxel_size_loop,5); 
        auto trans_icp = instanceIcp.get_trans(graph_q.front_points,graph_t.front_points,trans.matrix());

        PlaneIcp backgroundPlaceIcp(map_voxel_size_loop); 
        transform = backgroundPlaceIcp.getTransPlaneIcp(graph_q.back_points,graph_t.back_points,trans_icp);

        std::vector<int> match_node_label1;
        std::vector<int> match_node_label2;
        for(size_t i=0;i<refine_match_node_inGraph_q.second.size();i++){
            match_node_label1.emplace_back(graph_q.node_labels[refine_match_node_inGraph_q.second[i]]);
            match_node_label2.emplace_back(graph_t.node_labels[refine_match_node_inGraph_c.second[i]]);
        }
        match_instance_idx.assign(match_node_inGraph_q.second.begin(),match_node_inGraph_q.second.end());

        return true;
    }
    else{
        return false;
    }
}

/*
    Generating graph descriptors
*/
std::vector<float> GenGraphDescriptors(const Graph &graph, double edge_dis_th,int subinterval){
        float sub_interval_value = float(edge_dis_th/subinterval);
        std::vector<float> graph_desc(subinterval*6+3,0);

        for(size_t i=0; i<graph.edges.size();i++){
            if(graph.node_labels[graph.edges[i].first]==1 && graph.node_labels[graph.edges[i].second]==1){
                graph_desc[int(graph.edge_value[i]/sub_interval_value)+3]++;
            }
            else if(graph.node_labels[graph.edges[i].first]==2 && graph.node_labels[graph.edges[i].second]==2){
                graph_desc[subinterval*1+int(graph.edge_value[i]/sub_interval_value)+3]++;
            }
            else if(graph.node_labels[graph.edges[i].first]==3 && graph.node_labels[graph.edges[i].second]==3)
            {
                graph_desc[subinterval*2+int(graph.edge_value[i]/sub_interval_value)+3]++;
            }
            else if((graph.node_labels[graph.edges[i].first]==3 && graph.node_labels[graph.edges[i].second]==2)||(graph.node_labels[graph.edges[i].first]==2 && graph.node_labels[graph.edges[i].second]==3)){
                graph_desc[subinterval*3+int(graph.edge_value[i]/sub_interval_value)+3]++;
            }
            else if((graph.node_labels[graph.edges[i].first]==1 && graph.node_labels[graph.edges[i].second]==2)||(graph.node_labels[graph.edges[i].first]==2 && graph.node_labels[graph.edges[i].second]==1)){
                graph_desc[subinterval*4+int(graph.edge_value[i]/sub_interval_value)+3]++;
            }
            else if((graph.node_labels[graph.edges[i].first]==1 && graph.node_labels[graph.edges[i].second]==3)||(graph.node_labels[graph.edges[i].first]==3 && graph.node_labels[graph.edges[i].second]==1)){
                graph_desc[subinterval*5+int(graph.edge_value[i]/sub_interval_value)+3]++;
            }
        }

        double sum_of_squares_edge = std::inner_product(graph_desc.begin(), graph_desc.end(), graph_desc.begin(), 0.0f);
        float l2_norm_edge = float(std::sqrt(sum_of_squares_edge));
        if(l2_norm_edge==0) l2_norm_edge=0.000001f;
        for (auto it = graph_desc.begin(); it != graph_desc.end(); ++it) {
            *it /= l2_norm_edge; 
        }

        float sum_norm_l2 = float(std::sqrt(graph.vehicle_num*graph.vehicle_num + graph.trunk_num*graph.trunk_num + graph.pole_like_num*graph.pole_like_num)); 
        if(sum_norm_l2==0) sum_norm_l2=0.000001f;
        graph_desc[0] = ((float)graph.vehicle_num)/sum_norm_l2;
        graph_desc[1] = ((float)graph.trunk_num)/sum_norm_l2;
        graph_desc[2] = ((float)graph.pole_like_num)/sum_norm_l2;

        float sum_of_squares_all= std::inner_product(graph_desc.begin(), graph_desc.end(), graph_desc.begin(), 0.0f);
        float l2_norm_all = float(std::sqrt(sum_of_squares_all));
        if(l2_norm_all==0) l2_norm_all=0.000001f;
        for (auto it = graph_desc.begin(); it != graph_desc.end(); ++it) {
            *it /= l2_norm_all; 
        }

        return graph_desc;
    }

/*
    Generating back ground descriptors for scen descriptors
*/
std::vector<float> GenBackDescriptors(const V3d_i &filtered_pointcloud){
    auto ring_step = (max_dis - min_dis) / rings;
    auto sector_step = 360. / sectors;
    Eigen::MatrixXf Ssc_background_build = Eigen::MatrixXf::Zero(rings,sectors);
    Eigen::MatrixXf Ssc_background_road = Eigen::MatrixXf::Zero(rings,sectors);
    std::vector<float> background_dep;
    for(int i=0;i < (int)filtered_pointcloud.first.size(); i++){
        auto label = filtered_pointcloud.second[i];

        if (order_vec[label] > 0){

            double distance = std::sqrt(filtered_pointcloud.first[i][0] * filtered_pointcloud.first[i][0] + filtered_pointcloud.first[i][1] * filtered_pointcloud.first[i][1]);

            if (distance >= max_dis || distance < min_dis)
                {continue;}
            int sector_id = int(fastAtan2(filtered_pointcloud.first[i][1], filtered_pointcloud.first[i][0])/ sector_step);
            int ring_id = int((distance - min_dis) / ring_step);
            
            if (ring_id >= rings || ring_id < 0)
                {continue;}
            if (sector_id >= sectors || sector_id < 0)
                {continue;}
            if (label == 13) //build
            {
                Ssc_background_build(ring_id, sector_id) = 1;
            }
            else if (label == 9) //road
            {
                Ssc_background_road(ring_id, sector_id) = 1;
            }
        }
    }
    Eigen::VectorXf build_rowsum = Ssc_background_build.rowwise().sum();
    Eigen::VectorXf road_rowsum = Ssc_background_road.rowwise().sum();
    background_dep.insert(background_dep.end(),build_rowsum.data(),build_rowsum.data()+build_rowsum.size());
    background_dep.insert(background_dep.end(),road_rowsum.data(),road_rowsum.data()+road_rowsum.size());

    float sum_of_squares_node = std::inner_product(background_dep.begin(), background_dep.end(), background_dep.begin(), 0.0f);
    float l2_norm_node = std::sqrt(sum_of_squares_node);
    for (auto it = background_dep.begin(); it != background_dep.end(); ++it) {
        *it /= l2_norm_node; // 归一化
    }
    return background_dep;
}

/*
    Generating back ground descriptors for background similarity check
*/
Eigen::MatrixXf GenBackDescriptorsVeri(const V3d_i &filtered_pointcloud){

    auto ring_step = (max_dis - min_dis) / rings;
    auto sector_step = 360. / sectors;
    Eigen::MatrixXf Ssc_background = Eigen::MatrixXf::Zero(rings,sectors);
    
    for(int i=0;i < (int)filtered_pointcloud.first.size(); i++){
        auto label = filtered_pointcloud.second[i];
        if(label==14 || label==15) continue; 

        if (order_vec[label] > 0){
            
            double distance = std::sqrt(filtered_pointcloud.first[i][0]* filtered_pointcloud.first[i][0] + filtered_pointcloud.first[i][1] * filtered_pointcloud.first[i][1]);

            if (distance >= max_dis || distance < min_dis)
                {continue;}
            int sector_id = fastAtan2(filtered_pointcloud.first[i][1], filtered_pointcloud.first[i][0])/ sector_step;
            int ring_id = (distance - min_dis) / ring_step;

            if (ring_id >= rings || ring_id < 0)
                {continue;}
            if (sector_id >= sectors || sector_id < 0)
                {continue;}
            if (order_vec[label] > order_vec[Ssc_background(ring_id, sector_id)])
            {   
                Ssc_background(ring_id, sector_id) = label;
            }
        }
    }
    return Ssc_background;
}

/*
    Calculate cosine similarity between two descriptors
*/
double CalcaulateSim(const Eigen::MatrixXf &desc1, const Eigen::MatrixXf &desc2){
    double similarity = 0;
    int sectors_d = desc1.cols();
    int rings_d = desc1.rows();
    int valid_num = 0;
    for (int p = 0; p < sectors_d; p++)
    {
        for (int q = 0; q < rings_d; q++)
        {
            if (desc1(q, p) == 0 && desc2(q, p) == 0)
            {
                continue;
            }
            valid_num++;

            if (desc1(q, p) == desc2(q, p))
            {
                similarity++;
            }
        }
    }
    // std::cout<<similarity<<std::endl;
    return similarity / valid_num;
}

/*
    Find correspondences between two graphs using Hungarian algorithm
*/
std::tuple<V3d_i,V3d_i> FindCorrespondencesWithIdx(const Graph &graph1, const Graph &graph2){

    std::vector<std::vector<double>> associationCost_Matrix;
    associationCost_Matrix.resize(graph1.node_desc.size(),std::vector<double>(graph2.node_desc.size(),0));

    for(size_t i=0;i<graph1.node_desc.size();i++){
        for(size_t j=0;j<graph2.node_desc.size();j++){
            if(graph1.node_labels[i] == graph2.node_labels[j]){ // check node's label
                double node_des_cost = 1 - GetCosSim(graph1.node_desc[i],graph2.node_desc[j]); 
                if(node_des_cost > 0.5 || node_des_cost < 0) node_des_cost = 1e8;
                if(std::abs(graph1.node_dimensions[i].x()-graph2.node_dimensions[j].x())>2 ||
                        std::abs(graph1.node_dimensions[i].y()-graph2.node_dimensions[j].y())>2 ||
                            std::abs(graph1.node_dimensions[i].z()-graph2.node_dimensions[j].z())>2) node_des_cost = 1e8;
                associationCost_Matrix[i][j] = node_des_cost;
            }
            else{
                associationCost_Matrix[i][j] = 1e8;
            }
        }
    }
    // use Hungarian algorithm solve optimal correnpondences
    std::vector<int> assignment;
    HungarianAlgorithm HungAlgo;
    assignment.clear();
    HungAlgo.Solve(associationCost_Matrix,assignment);

    // match results
    std::vector<Eigen::Vector3d> query_nodes_center;
    std::vector<Eigen::Vector3d> match_nodes_center;
    std::vector<int> query_nodes_idx;
    std::vector<int> match_nodes_idx;
    for(size_t i=0;i<graph1.node_desc.size();i++){
        if(assignment[i]!=-1 && associationCost_Matrix[i][assignment[i]]<1e8){
            query_nodes_center.emplace_back(graph1.node_centers[i]);
            query_nodes_idx.emplace_back(i);
            match_nodes_center.emplace_back(graph2.node_centers[assignment[i]]);
            match_nodes_idx.emplace_back(assignment[i]);
        }
    }
    return {std::make_pair(query_nodes_center,query_nodes_idx),std::make_pair(match_nodes_center,match_nodes_idx)};
}

/*
    Find correspondences between two graphs using KDtree search
*/
std::tuple<V3d_i,V3d_i> FindCorrespondencesKDtree(const Graph &graph1, const Graph &graph2, int search_results_num){

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
            // outlier pruning is proposed in SGLC
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


/*
    Outlier instance node pruning based on local geometry structure
*/
std::tuple<V3d_i,V3d_i> OutlierPruning(const Graph &graph1, const Graph &graph2, V3d_i match_node1, V3d_i match_node2){
    
    std::vector<Eigen::Vector3d> inlier_match_node1;
    std::vector<Eigen::Vector3d> inlier_match_node2;

    std::vector<int> inlier_match_node1_idx;
    std::vector<int> inlier_match_node2_idx;

    float subgraph_edge_th = 20;
    if(match_node1.first.size()<15) { //15
        inlier_match_node1 = match_node1.first;
        inlier_match_node1_idx = match_node1.second;
        inlier_match_node2 = match_node2.first;
        inlier_match_node2_idx = match_node2.second;
        return {std::make_pair(inlier_match_node1,inlier_match_node1_idx),std::make_pair(inlier_match_node2,inlier_match_node2_idx)};
    }
    
    assert(match_node1.first.size()==match_node2.first.size());
    for(size_t i=0;i<match_node1.first.size();i++){
        
        int matche_sub_Triangle_th=0;
        std::vector<Eigen::Vector3d> nodeSubgraphTriangle1;
        std::vector<Eigen::Vector3d> nodeSubgraphTriangle2;
        
        // node in graph1
        auto edge_list1 = graph1.edge_matrix.row(match_node1.second[i]);
        std::vector<int> indices1;
        for(int m=0;m<edge_list1.size();m++){
            if(edge_list1[m]<subgraph_edge_th && edge_list1[m]!=0) indices1.push_back(m);
        }

        if(indices1.size()<2) { 
                // inlier_match_node1.emplace_back(match_node1.first[i]);
                // inlier_match_node2.emplace_back(match_node2.first[i]);
                continue;
        }
        for(size_t m=0; m<indices1.size();m++){
            if(m==(indices1.size()-1)) break;
            for(size_t n=m+1; n<indices1.size();n++){
                std::vector<float> sub_triangle(3);
                sub_triangle[0] = (float)(match_node1.first[i]-graph1.node_centers[indices1[m]]).norm();
                sub_triangle[1] = (float)(match_node1.first[i]-graph1.node_centers[indices1[n]]).norm();
                sub_triangle[2] = (float)(graph1.node_centers[indices1[m]]-graph1.node_centers[indices1[n]]).norm();
                std::sort(sub_triangle.begin(), sub_triangle.end()); //边从小到大排序
                nodeSubgraphTriangle1.emplace_back(Eigen::Vector3d(sub_triangle[0],sub_triangle[1],sub_triangle[2]));
            }
        }
        // node in graph2
        auto edge_list2 = graph2.edge_matrix.row(match_node2.second[i]);
        std::vector<int> indices2;
        for(int m=0;m<edge_list2.size();m++){
            if(edge_list2[m]<subgraph_edge_th && edge_list2[m]!=0) indices2.push_back(m);
        }

        if(indices2.size()<2) { 
                // inlier_match_node1.emplace_back(match_node1.first[i]);
                // inlier_match_node2.emplace_back(match_node2.first[i]);
                continue;
        }
        for(size_t m=0; m<indices2.size();m++){
            if(m==(indices2.size()-1)) break;
            for(size_t n=m+1; n<indices2.size();n++){
                std::vector<float> sub_triangle(3);
                sub_triangle[0] = (float)(match_node2.first[i]-graph2.node_centers[indices2[m]]).norm();
                sub_triangle[1] = (float)(match_node2.first[i]-graph2.node_centers[indices2[n]]).norm();
                sub_triangle[2] = (float)(graph2.node_centers[indices2[m]]-graph2.node_centers[indices2[n]]).norm();
                std::sort(sub_triangle.begin(), sub_triangle.end()); //边从小到大排序
                nodeSubgraphTriangle2.emplace_back(Eigen::Vector3d(sub_triangle[0],sub_triangle[1],sub_triangle[2]));
            }
        }

        // match subgraph triangles
        for(size_t m=0;m<nodeSubgraphTriangle1.size();m++){
            for(size_t n=0;n<nodeSubgraphTriangle2.size();n++){
                if(abs(nodeSubgraphTriangle1[m].x() - nodeSubgraphTriangle2[n].x())<0.3&&
                    abs(nodeSubgraphTriangle1[m].y() - nodeSubgraphTriangle2[n].y())<0.3&&
                    abs(nodeSubgraphTriangle1[m].z() - nodeSubgraphTriangle2[n].z())<0.3){
                        matche_sub_Triangle_th++;
                }
            }
        }

        if(indices1.size()>3 && indices2.size()>3){
            if(matche_sub_Triangle_th>=2) { 
                inlier_match_node1.emplace_back(match_node1.first[i]);
                inlier_match_node1_idx.emplace_back(match_node1.second[i]);
                inlier_match_node2.emplace_back(match_node2.first[i]);
                inlier_match_node2_idx.emplace_back(match_node2.second[i]);
            }
        }
        else{
            if(matche_sub_Triangle_th>=1) {
                inlier_match_node1.emplace_back(match_node1.first[i]);
                inlier_match_node1_idx.emplace_back(match_node1.second[i]);
                inlier_match_node2.emplace_back(match_node2.first[i]);
                inlier_match_node2_idx.emplace_back(match_node2.second[i]);
            }
        }
        

    }
    return {std::make_pair(inlier_match_node1,inlier_match_node1_idx),std::make_pair(inlier_match_node2,inlier_match_node2_idx)};
}

/*
    Check if two subgraph triangles are matched
*/
bool CheckSubTriangle(const std::vector<Eigen::Vector3d> &nodeSubgraphTriangle1, const std::vector<Eigen::Vector3d> &nodeSubgraphTriangle2){

    int matche_sub_Triangle_th = 0;
    bool sub_triangle_match = false;
    if(nodeSubgraphTriangle1.size()<2 && nodeSubgraphTriangle2.size()<2) return true; 

    for(size_t m=0;m<nodeSubgraphTriangle1.size();m++){
        for(size_t n=0;n<nodeSubgraphTriangle2.size();n++){
            if(abs(nodeSubgraphTriangle1[m].x() - nodeSubgraphTriangle2[n].x())<0.3&&
                abs(nodeSubgraphTriangle1[m].y() - nodeSubgraphTriangle2[n].y())<0.3&&
                abs(nodeSubgraphTriangle1[m].z() - nodeSubgraphTriangle2[n].z())<0.3){
                    matche_sub_Triangle_th++;
            }
        }
    }
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


/*
    RANSAC alignment
*/
std::tuple<Eigen::Isometry3d,double> RansacAlignment(const std::vector<Eigen::Vector3d> &match_node1,
                                                     const std::vector<Eigen::Vector3d> &match_node2,
                                                    const std::vector<Eigen::Vector3d> &node1,
                                                    const std::vector<Eigen::Vector3d> &node2,
                                                    int max_inter,  int& best_inlier_num){
    assert(match_node1.size()==match_node2.size());
    std::random_device rd;
    std::mt19937 gen(rd());
            
    std::uniform_int_distribution<> dis(0, match_node1.size()-1);
    
    // int best_inlier_num = 0;
    double final_loss = 1000000.0;
    Eigen::Isometry3d final_tans = Eigen::Isometry3d::Identity();


    for(int i=0;i<max_inter;i++){
        int inlier_num =0;
        double loss = 0.0;
        int select_index1 = dis(gen);
        int select_index2 = dis(gen);
        int select_index3 = dis(gen);

        std::vector<Eigen::Vector3d> select_node1(3);
        std::vector<Eigen::Vector3d> select_node2(3);

        select_node1[0] = match_node1[select_index1];
        select_node1[1] = match_node1[select_index2];
        select_node1[2] = match_node1[select_index3];

        select_node2[0] = match_node2[select_index1];
        select_node2[1] = match_node2[select_index2];
        select_node2[2] = match_node2[select_index3];

        // SVD decomposition
        Eigen::Isometry3d trans_matrix = SolveSVD(select_node1,select_node2);

        float voxel_size = 0.2;
        tsl::robin_map<Eigen::Vector3i, VoxelBlock, VoxelHash> node2_voxel_map;
        
        std::for_each(node2.cbegin(), node2.cend(), [&](const Eigen::Vector3d &point) {
            auto voxel = Eigen::Vector3i((point/voxel_size).template cast<int>());
            auto search = node2_voxel_map.find(voxel);
            if (search != node2_voxel_map.end()) {
                auto &voxel_block = search.value();
                voxel_block.AddPoint(point);
            } else {
                node2_voxel_map.insert({voxel, VoxelBlock{{point}}});
            }
        });

        for( size_t i=0;i< node1.size();i++){
            Eigen::Vector3d trans_match_node1 = trans_matrix*node1[i];
            
            auto kx = static_cast<int>(trans_match_node1[0] / voxel_size);
            auto ky = static_cast<int>(trans_match_node1[1] / voxel_size);
            auto kz = static_cast<int>(trans_match_node1[2] / voxel_size);
            std::vector<Eigen::Vector3i> voxels;
            voxels.reserve(9);
            for (int i = kx - 1; i < kx + 1 + 1; ++i) {
                for (int j = ky - 1; j < ky + 1 + 1; ++j) {
                    for (int k = kz - 1; k < kz + 1 + 1; ++k) {
                            voxels.emplace_back(i, j, k);
                    }
                }
            }
            
            std::vector<Eigen::Vector3d> neighboors;
            std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
                auto search = node2_voxel_map.find(voxel);
                if (search != node2_voxel_map.end()) {
                    const auto &points = search->second.points;
                    if (!points.empty()) {
                            neighboors.emplace_back(points[0]);
                        }
                    }
            });
            if(neighboors.size()>0){
                double trans_node_dis = (neighboors[0] - trans_match_node1).norm();
                inlier_num++;
                loss = loss + trans_node_dis;
            }


        }

        if(inlier_num>best_inlier_num){
            best_inlier_num = inlier_num;
            final_tans = trans_matrix;
            final_loss = loss/inlier_num;
            if((((float)inlier_num/node1.size())>0.5 && inlier_num>2)||inlier_num>8){ 
                break;
            }
            
        }
    }
    if(best_inlier_num<3) final_loss=1000000;
    double score = exp(-final_loss);
    return {final_tans,score};
}

/*
    RAANSAC alignment
*/
std::tuple<Eigen::Isometry3d,double> RansacAlignment(std::vector<Eigen::Vector3d> match_node1, std::vector<Eigen::Vector3d> match_node2, 
                                                    const Graph &graph1, const Graph &graph2, int max_inter, int& best_inlier_num){
    
    assert(match_node1.size()==match_node2.size());
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_int_distribution<> dis(0, match_node1.size()-1);

    Eigen::Isometry3d final_tans = Eigen::Isometry3d::Identity();

    float voxel_size = 0.2;
    tsl::robin_map<Eigen::Vector3i, VoxelBlock, VoxelHash> node2_voxel_map;
    
    std::for_each(graph2.node_centers.cbegin(), graph2.node_centers.cend(), [&](const Eigen::Vector3d &point) {
        auto voxel = Eigen::Vector3i((point/voxel_size).template cast<int>());
        auto search = node2_voxel_map.find(voxel);
        if (search != node2_voxel_map.end()) {
            auto &voxel_block = search.value();
            voxel_block.AddPoint(point);
        } else {
            node2_voxel_map.insert({voxel, VoxelBlock{{point}}});
        }
    });

    for(int i=0;i<max_inter;i++){
        int inlier_num =0;
        double loss = 0.0;

        // ransac
        int select_index1 = dis(gen);
        int select_index2 = dis(gen);
        int select_index3 = dis(gen);

        std::vector<Eigen::Vector3d> select_node1(3);
        std::vector<Eigen::Vector3d> select_node2(3);

        select_node1[0] = match_node1[select_index1];
        select_node1[1] = match_node1[select_index2];
        select_node1[2] = match_node1[select_index3];

        select_node2[0] = match_node2[select_index1];
        select_node2[1] = match_node2[select_index2];
        select_node2[2] = match_node2[select_index3];

        // svd decomposition
        Eigen::Isometry3d trans_matrix = SolveSVD(select_node1,select_node2);

        // find corresponding node in graph2, select inlier points
        for( size_t i=0;i< graph1.node_centers.size();i++){
            Eigen::Vector3d trans_match_node1 = trans_matrix*graph1.node_centers[i];
            
            auto kx = static_cast<int>(trans_match_node1[0] / voxel_size);
            auto ky = static_cast<int>(trans_match_node1[1] / voxel_size);
            auto kz = static_cast<int>(trans_match_node1[2] / voxel_size);
            std::vector<Eigen::Vector3i> voxels;
            voxels.reserve(9);
            for (int i = kx - 1; i < kx + 1 + 1; ++i) {
                for (int j = ky - 1; j < ky + 1 + 1; ++j) {
                    for (int k = kz - 1; k < kz + 1 + 1; ++k) {
                            voxels.emplace_back(i, j, k);
                    }
                }
            }
            
            std::vector<Eigen::Vector3d> neighboors;
            std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
                auto search = node2_voxel_map.find(voxel);
                if (search != node2_voxel_map.end()) {
                    const auto &points = search->second.points;
                    if (!points.empty()) {
                            neighboors.emplace_back(points[0]);
                        }
                    }
            });
            if(neighboors.size()>0){
                double trans_node_dis = (neighboors[0] - trans_match_node1).norm();
                inlier_num++;
                loss = loss + trans_node_dis;
            }
        }

        if(inlier_num>best_inlier_num){
            best_inlier_num = inlier_num;
            final_tans = trans_matrix;
        }
    }
    double score = (float)best_inlier_num/graph1.node_centers.size();
    return {final_tans,score};
}

/*
    RAANSAC alignment
*/
std::tuple<Eigen::Isometry3d,double> RansacAlignment(std::vector<Eigen::Vector3d> match_node1, std::vector<Eigen::Vector3d> match_node2,int max_inter, int& best_inlier_num){
    assert(match_node1.size()==match_node2.size());
    std::random_device rd;
    std::mt19937 gen(rd());
            
    std::uniform_int_distribution<> dis(0, match_node1.size()-1);
    
    // int best_inlier_num = 0;
    double final_loss = 1000000.0;
    Eigen::Isometry3d final_tans = Eigen::Isometry3d::Identity();

    for(int i=0;i<max_inter;i++){
        int inlier_num =0;
        double loss = 0.0;

        int select_index1 = dis(gen);
        int select_index2 = dis(gen);
        int select_index3 = dis(gen);

        std::vector<Eigen::Vector3d> select_node1(3);
        std::vector<Eigen::Vector3d> select_node2(3);

        select_node1[0] = match_node1[select_index1];
        select_node1[1] = match_node1[select_index2];
        select_node1[2] = match_node1[select_index3];

        select_node2[0] = match_node2[select_index1];
        select_node2[1] = match_node2[select_index2];
        select_node2[2] = match_node2[select_index3];

        Eigen::Isometry3d trans_matrix = SolveSVD(select_node1,select_node2);

        for( size_t i=0;i< match_node1.size();i++){
            double trans_node_dis = (trans_matrix*match_node1[i] - match_node2[i]).norm();
            if(trans_node_dis<0.2){  // 30cm内就算一个inlier
                inlier_num++;
                loss = loss + trans_node_dis;
            }
        }
        if(inlier_num>best_inlier_num){
            best_inlier_num = inlier_num;
            final_tans = trans_matrix;
            final_loss = loss/inlier_num;
        }
    }
    if(best_inlier_num<3) final_loss=1000000;

    double score = exp(-final_loss);
    return {final_tans,score};
}

/*
    SVD decomposition
*/
Eigen::Isometry3d SolveSVD(const std::vector<Eigen::Vector3d> &match_node1,
                            const std::vector<Eigen::Vector3d> &match_node2){

    if(match_node1.empty() || match_node2.empty() || match_node1.size()!=match_node2.size()){
        std::cout<<"Error! solve SVD: input pointcloud size is not same or empty"<<std::endl;
    }

    int N=match_node1.size();
    Eigen::Vector3d node1_sum{0,0,0}, node2_sum{0,0,0};
    for(int i=0;i<N;i++){
        node1_sum+=match_node1[i];
        node2_sum+=match_node2[i];
    }
    Eigen::Vector3d node1_mean = node1_sum/N;
    Eigen::Vector3d node2_mean = node2_sum/N;

    std::vector<Eigen::Vector3d> node1_list,node2_list;
    for(int i=0;i<N;i++){
        node1_list.emplace_back(match_node1[i]-node1_mean);
        node2_list.emplace_back(match_node2[i]-node2_mean);
    }

    // calculate W = q1*q2^t
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i=0;i<N;i++){
        W += node1_list[i] * node2_list[i].transpose();
    }
    for (int i = 0; i < W.size(); i++) {
        if (std::isnan(W(i))) {
            std::cout << "error: the input points are wrong, can't solve with SVD." << std::endl;
        }
    }

    // svd 
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d E = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_temp = V*(U.transpose());

    if (R_temp.determinant() < 0) {
        E(2,2) = -1;
    }
    Eigen::Matrix3d R = V * E * (U.transpose());
    Eigen::Vector3d T = node2_mean - R * node1_mean;

    Eigen::Isometry3d trans_matrix = Eigen::Isometry3d::Identity();
    trans_matrix.linear() = R;
    trans_matrix.translation() = T;   

    return  trans_matrix;
}





// claculate cosine similiraty
double GetCosSim(const std::vector<float> vec1, const std::vector<float> vec2){
    assert(vec1.size()==vec2.size());
    double tmp= 0.0;
    for(size_t i=0;i<vec1.size();i++){
        tmp += vec1[i]*vec2[i];
    }
    double simility = tmp / (GetMold(vec1)*GetMold(vec2));
    return simility;
}

double GetMold(const std::vector<int> vec){
    double sum = 0.0;
    for(size_t i=0;i<vec.size();i++){
        sum += vec[i]*vec[i];
    }
    if(sum==0) sum = 0.00000000001; 
    return std::sqrt(sum);
}

double GetMold(const std::vector<float> vec){
    double sum = 0.0;
    for(size_t i=0;i<vec.size();i++){
        sum += vec[i]*vec[i];
    }
    if(sum==0) sum = 0.00000000001; 
    return std::sqrt(sum);
}

double fastAtan2(const double &y, const double &x)
{
    double ax = std::abs(x), ay = std::abs(y);
    double a, c, c2;
    if( ax >= ay )
    {
        c = ay/(ax + (double)DBL_EPSILON);
        c2 = c*c;
        a = (((atan2_p7*c2 + atan2_p5)*c2 + atan2_p3)*c2 + atan2_p1)*c;
    }
    else
    {
        c = ax/(ay + (double)DBL_EPSILON);
        c2 = c*c;
        a = 90.f - (((atan2_p7*c2 + atan2_p5)*c2 + atan2_p3)*c2 + atan2_p1)*c;
    }
    if( x < 0 )  a = 180.f - a;
    if( y < 0 )  a = 360.f - a;
    return a;
}

}