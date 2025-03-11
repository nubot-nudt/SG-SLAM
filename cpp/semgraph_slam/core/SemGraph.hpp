// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <numeric> 

#include "Coreutils.h"


namespace graph_slam{


Eigen::MatrixXd MatrixDecomposing(Eigen::MatrixXd MatrixInput,int Dimension);
Graph BuildGraph(const std::vector<Bbox> &cluster_boxes, double edge_dis_th,int subinterval,int graph_node_dimension,double subgraph_edge_th);
Graph ReBuildGraph(const std::vector<InsNode> &cluster_boxes, double edge_dis_th,int subinterval,int graph_node_dimension,double subgraph_edge_th);
}