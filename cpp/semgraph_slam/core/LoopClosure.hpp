// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>


#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <numeric>
#include <cmath>
#include <cfloat>
#include <random>
#include <tsl/robin_map.h>

#include "Coreutils.h"
#include "Hungarian.hpp"
#include "FastIcp.hpp"
#include "PlaneIcp.hpp"
#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

#define CV_PI   3.1415926535897932384626433832795

namespace graph_slam{


// struct VoxelHash{
//             size_t operator()(const Eigen::Vector3i &voxel) const {
//                 const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
//                 return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
//                 }
// }; 
using NodeDesTree = KDTreeVectorOfVectorsAdaptor< std::vector<std::vector<float>>, float >;

struct VoxelBlock {
        std::vector<Eigen::Vector3d> points;
        inline void AddPoint(const Eigen::Vector3d &point) {
            points.push_back(point);
        }
};

static const double atan2_p1 = 0.9997878412794807f*(double)(180/CV_PI);
static const double atan2_p3 = -0.3258083974640975f*(double)(180/CV_PI);
static const double atan2_p5 = 0.1555786518463281f*(double)(180/CV_PI);
static const double atan2_p7 = -0.04432655554792128f*(double)(180/CV_PI);

const double max_dis=50;
const double min_dis=5;
const int rings=24;
const int sectors=360;

const std::vector<int> order_vec = {0, 22, 0, 0, 21, 20, 0, 0, 0, 10, 11, 12, 13, 15, 16, 14, 17, 9, 18, 19, 0, 0, 0, 0, 0, 0};

std::vector<float> GenScanDescriptors(const Graph &graph,double edge_dis_th,int subinterval);
std::vector<float> GenGraphDescriptors(const Graph &graph, double edge_dis_th,int subinterval);
std::vector<float> GenBackDescriptors(const V3d_i &filtered_pointcloud);
Eigen::MatrixXf GenBackDescriptorsVeri(const V3d_i &filtered_pointcloud);

bool GeometryVeriPoseEstimation(const Graph &graph_q, const Graph &graph_t, Eigen::Matrix4d &transform,std::vector<int>& match_instance_idx,
                                double graph_sim_th,double back_sim_th,double map_voxel_size_loop);
std::tuple<V3d_i,V3d_i> FindCorrespondencesWithIdx(const Graph &graph1, const Graph &graph2);
std::tuple<V3d_i,V3d_i> FindCorrespondencesKDtree(const Graph &graph1, const Graph &graph2, int search_results_num);
std::tuple<V3d_i,V3d_i> OutlierPruning(const Graph &graph1, const Graph &graph2, V3d_i match_node1, V3d_i match_node2);
bool CheckSubTriangle(const std::vector<Eigen::Vector3d> &nodeSubgraphTriangle1, const std::vector<Eigen::Vector3d> &nodeSubgraphTriangle2);
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
double fastAtan2( const double &y, const double &x);
double GetCosSim(const std::vector<float> vec1, const std::vector<float> vec2);
double CalcaulateSim(const Eigen::MatrixXf &desc1, const Eigen::MatrixXf &desc2);
double GetMold(const std::vector<int> vec);
double GetMold(const std::vector<float> vec);

}