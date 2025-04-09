// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>


#pragma once

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tsl/robin_map.h>

#include "semgraph_slam/core/Coreutils.h"
#include "semgraph_slam/core/nanoflann.hpp"
#include "semgraph_slam/core/KDTreeVectorOfVectorsAdaptor.h"
#include "semgraph_slam/core/CommonUtils.hpp"

#include "SemGraph.hpp"

namespace graph_slam{

using VTbii = std::vector<std::tuple<bool,int,int>>;  // (imatch flag, instance id in current graph, instnace in local map)

std::tuple<V3d_i,V3d_i> FindCorrespondencesKDtree(const Graph &graph1, const Graph &graph2, int search_results_num);
bool CheckSubTriangle(const std::vector<Eigen::Vector3d> &nodeSubgraphTriangle1, const std::vector<Eigen::Vector3d> &nodeSubgraphTriangle2);

struct GraphMap {

    int instance_id_ = 0;
    double voxel_size_ = 1.5; // for finging instance

    double edge_dis_th_ = 60;
    double subgraph_edge_th_ = 20;
    int subinterval_ = 30;
    int graph_node_dimension_ = 30;

    double nearest_neighbor_vehicle_disth_ = 1.0; // for finding correspondences
    double nearest_neighbor_pole_disth_ = 1.0;

    double max_distance_ = 100.0;

    std::pair<V3d,V3d> relo_corr;

    struct IndexBlock{
        std::vector<int> index_instance;
        inline void AddIndex(int index) {index_instance.push_back(index);}
    };

    Graph InsGraphMap;
    std::vector<InsNode> instance_in_localmap;
    VTbii FindInsMatch(const Graph &graph);
    std::tuple< Sophus::SE3d, bool> Relocalization(const Graph &graph,const VTbii &frame2map_match,double inlier_rate_th);
    explicit GraphMap(double edge_dis_th, double subgraph_edge_th,int subinterval, int graph_node_dimension, double neb_vehicle_disth, double neb_pole_disth,double max_distance)
            :  edge_dis_th_(edge_dis_th),
               subgraph_edge_th_(subgraph_edge_th),
               subinterval_(subinterval),
               graph_node_dimension_(graph_node_dimension),
               nearest_neighbor_vehicle_disth_(neb_vehicle_disth),
               nearest_neighbor_pole_disth_(neb_pole_disth),
               max_distance_(max_distance) {}


    void Update(Graph &graph, const VTbii &frame2map_match, const Sophus::SE3d &pose);

    bool IsExistNearestNeighbor(const tsl::robin_map<Eigen::Vector3i, IndexBlock, VoxelHash>& instance_poses_map, const Eigen::Vector3d & poses_ins, int labels, double neighbor_th);

    // void EraseIndexInMap(const int &index);

    // tsl::robin_map<Eigen::Vector3i, IndexBlock, VoxelHash> instance_poses_map;  // voxel -> index in InsGraphMap
};


} // namespace graph_slam