
#include "CommonUtils.hpp"

namespace graph_slam{


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

}