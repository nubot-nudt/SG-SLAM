// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#pragma once

#include <Eigen/Core>
#include <vector>

namespace graph_slam{

// Color codes
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


// Data types
typedef std::pair<std::vector<Eigen::Vector3d>,std::vector<int>> V3d_i;
typedef std::vector<Eigen::Vector4d> V4d;
typedef  std::vector<Eigen::Vector3d> V3d;


// Hash function for Eigen::Vector3i
struct VoxelHash{
            size_t operator()(const Eigen::Vector3i &voxel) const {
                const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
                return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
                }
}; 

// Data structure for bounding box
struct Bbox
{
    Eigen::Vector3d center;
    Eigen::Vector3d dimension;
    double theta = 0.0;
    int label = -1;
    double score = 0.0;
    int points_num;
};

// Data structure for graph
struct Graph
{
    std::vector<int> node_labels;
    std::vector<int> points_num;
    std::vector<Eigen::Vector3d> node_centers;
    std::vector<std::vector<Eigen::Vector3d>> node_sub_triangles; 
    std::vector<Eigen::Vector3d> node_dimensions;
    std::vector<std::vector<float>> node_desc; 
    std::vector<std::pair<int,int>> edges;
    std::vector<int> new_instance;
    std::vector<double> edge_value;
    std::vector<double> edge_weights;
    Eigen::MatrixXd graph_matrix;
    Eigen::MatrixXd edge_matrix;
    V3d_i back_points;
    V3d_i front_points;
    int vehicle_num=0;
    int trunk_num=0;
    int pole_like_num=0;
};

// Data structure for instance node in local map
struct InsNode{
    int label;
    int id;
    Eigen::Vector3d pose;
    Eigen::Vector3d dimension;
    int points_num;
};

}


