#pragma once

#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numeric>
#include <cmath>
#include <cfloat>
#include <random>
#include <tsl/robin_map.h>


#include "Coreutils.h"

namespace graph_slam{

struct VoxelBlock {
        std::vector<Eigen::Vector3d> points;
        inline void AddPoint(const Eigen::Vector3d &point) {
            points.push_back(point);
        }
};

Eigen::Isometry3d SolveSVD(const std::vector<Eigen::Vector3d> &match_node1,
        const std::vector<Eigen::Vector3d> &match_node2);
std::tuple<Eigen::Isometry3d,double> RansacAlignment(const std::vector<Eigen::Vector3d> &match_node1,
                                 const std::vector<Eigen::Vector3d> &match_node2,
                                const std::vector<Eigen::Vector3d> &node1,
                                const std::vector<Eigen::Vector3d> &node2,
                                int max_inter,  int& best_inlier_num);
std::tuple<Eigen::Isometry3d,double> RansacAlignment(std::vector<Eigen::Vector3d> match_node1, 
                                std::vector<Eigen::Vector3d> match_node2, 
                                const Graph &graph1,
                                const Graph &graph2,
                                int max_inter, int& best_inlier_num);
std::tuple<Eigen::Isometry3d,double> RansacAlignment(std::vector<Eigen::Vector3d> match_node1,
                                std::vector<Eigen::Vector3d> match_node2,
                                int max_inter, int& best_inlier_num);

}