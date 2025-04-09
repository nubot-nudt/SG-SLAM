// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#include "SemGraph.hpp" 


namespace graph_slam{



/*
    Building semantic graph from clustered bounding boxes
*/
Graph BuildGraph(const std::vector<Bbox> &cluster_boxes, double edge_dis_th,int subinterval,int graph_node_dimension,double subgraph_edge_th){
    Graph frame_graph;
    int sub_interval_value = int(edge_dis_th/subinterval);
    size_t N = cluster_boxes.size();
    // float subgraph_edge_th = 20; // 20m for sub triangle edge threshold
    Eigen::MatrixXd AdjacencyMatrix = Eigen::MatrixXd::Zero(N,N);    
    Eigen::MatrixXd EdgeMatrix = Eigen::MatrixXd::Zero(N,N);
    Eigen::MatrixXd NodeEmbeddings = Eigen::MatrixXd::Zero(N,subinterval*3+graph_node_dimension);
    Eigen::MatrixXd NodeEmbeddings_Local = Eigen::MatrixXd::Zero(N,subinterval*3);

    for(size_t i = 0; i < N; i++){
        frame_graph.node_labels.emplace_back(cluster_boxes[i].label);  // node labels
        frame_graph.node_centers.emplace_back(cluster_boxes[i].center); // node centers
        frame_graph.node_dimensions.emplace_back(cluster_boxes[i].dimension); // node bounding box dimensions
        frame_graph.points_num.emplace_back(cluster_boxes[i].points_num); // node points number
        

        if(cluster_boxes[i].label==1) frame_graph.vehicle_num = frame_graph.vehicle_num+1;
        else if(cluster_boxes[i].label==2) frame_graph.trunk_num = frame_graph.trunk_num+1;
        else if(cluster_boxes[i].label==3) frame_graph.pole_like_num = frame_graph.pole_like_num+1;

        std::vector<std::pair<int,double>> vertex_edges;
        for(size_t j = 0; j< N; j++){
            double edge = (cluster_boxes[i].center - cluster_boxes[j].center).norm();
            if(edge<edge_dis_th){
                vertex_edges.emplace_back(std::make_pair(cluster_boxes[j].label,edge));
                AdjacencyMatrix(i,j) = 1;
                EdgeMatrix(i,j) = edge;
                if(j>=i+1){                                          // only count once
                    frame_graph.edges.emplace_back(std::make_pair(i,j));
                    frame_graph.edge_value.emplace_back(edge);
                    frame_graph.edge_weights.emplace_back((edge_dis_th-edge)/edge_dis_th); //[0,edge_dis_th]->[1,0]
                }
            }
        }
        // build vertes desc: main for finding node correspondence
        for(size_t m=0;m<vertex_edges.size();m++){
            if(vertex_edges[m].first == 1){ // x - vehicle
                NodeEmbeddings_Local(i,int(vertex_edges[m].second/sub_interval_value))++;
            }
            else if(vertex_edges[m].first == 2){ // x - truck
                NodeEmbeddings_Local(i,subinterval+int(vertex_edges[m].second/sub_interval_value))++;
            }
            else if(vertex_edges[m].first == 3){ // x - pole
                NodeEmbeddings_Local(i,subinterval*2+int(vertex_edges[m].second/sub_interval_value))++;
            }
        }

    }

    if(frame_graph.node_labels.size()==0) return  frame_graph;
    // only 0.0xms -> 0.1ms 
    Eigen::MatrixXd NodeEmbeddings_Global= MatrixDecomposing(AdjacencyMatrix,graph_node_dimension);
    NodeEmbeddings_Local = NodeEmbeddings_Local.array().colwise()/NodeEmbeddings_Local.rowwise().norm().array();
    NodeEmbeddings_Global = NodeEmbeddings_Global.array().colwise()/NodeEmbeddings_Global.rowwise().sum().array();
    NodeEmbeddings.leftCols(subinterval*3) = NodeEmbeddings_Local;
    NodeEmbeddings.rightCols(graph_node_dimension) = NodeEmbeddings_Global;

    for(size_t i=0;i<N;i++){
        Eigen::MatrixXd evec_sort_row = NodeEmbeddings.row(i);
        std::vector<float> node_desf(evec_sort_row.data(),evec_sort_row.data()+evec_sort_row.size());
        frame_graph.node_desc.emplace_back(node_desf);  // node descriptors
    }


    // build local sub triangle for outlier pruning
    for(size_t i=0;i<N;i++){
        frame_graph.node_sub_triangles.emplace_back(std::vector<Eigen::Vector3d>());
        std::vector<int> indices;
        std::vector<Eigen::Vector3d> nodeSubgraphTriangle;
        auto edge_list = EdgeMatrix.row(i);
        for(int m=0;m<edge_list.size();m++){
            if(edge_list[m]<subgraph_edge_th && edge_list[m]!=0) indices.push_back(m);
        }
        if(indices.size()>2) {
            for(size_t m=0; m<indices.size();m++){
                if(m==(indices.size()-1)) break;
                for(size_t n=m+1; n<indices.size();n++){
                    std::vector<float> sub_triangle(3);
                    sub_triangle[0] = (float)(frame_graph.node_centers[i]-frame_graph.node_centers[indices[m]]).norm();
                    sub_triangle[1] = (float)(frame_graph.node_centers[i]-frame_graph.node_centers[indices[n]]).norm();
                    sub_triangle[2] = (float)(frame_graph.node_centers[indices[m]]-frame_graph.node_centers[indices[n]]).norm();
                    std::sort(sub_triangle.begin(), sub_triangle.end()); // sort triangle edges
                    nodeSubgraphTriangle.emplace_back(Eigen::Vector3d(sub_triangle[0],sub_triangle[1],sub_triangle[2]));
                }
            }
            frame_graph.node_sub_triangles[i].assign(nodeSubgraphTriangle.begin(),nodeSubgraphTriangle.end());
        }
        
    }

    frame_graph.edge_matrix = EdgeMatrix; // for subsequent correspondences pruning

    return frame_graph;
}

/*
    Rebuilding graph for local graph map
*/
Graph ReBuildGraph(const std::vector<InsNode> &cluster_boxes, double edge_dis_th,int subinterval,int graph_node_dimension,double subgraph_edge_th){
    Graph frame_graph;
    int sub_interval_value = int(edge_dis_th/subinterval);
    size_t N = cluster_boxes.size();
    // float subgraph_edge_th = 20; // 20m for sub triangle edge threshold
    Eigen::MatrixXd AdjacencyMatrix = Eigen::MatrixXd::Zero(N,N);
    Eigen::MatrixXd EdgeMatrix = Eigen::MatrixXd::Zero(N,N);
    Eigen::MatrixXd NodeEmbeddings = Eigen::MatrixXd::Zero(N,subinterval*3+graph_node_dimension);
    Eigen::MatrixXd NodeEmbeddings_Local = Eigen::MatrixXd::Zero(N,subinterval*3);

    for(size_t i = 0; i < N; i++){
        frame_graph.node_labels.emplace_back(cluster_boxes[i].label);
        frame_graph.node_centers.emplace_back(cluster_boxes[i].pose);
        frame_graph.node_dimensions.emplace_back(cluster_boxes[i].dimension);
        frame_graph.points_num.emplace_back(cluster_boxes[i].points_num);
        

        if(cluster_boxes[i].label==1) frame_graph.vehicle_num = frame_graph.vehicle_num+1;
        else if(cluster_boxes[i].label==2) frame_graph.trunk_num = frame_graph.trunk_num+1;
        else if(cluster_boxes[i].label==3) frame_graph.pole_like_num = frame_graph.pole_like_num+1;

        std::vector<std::pair<int,double>> vertex_edges;
        for(size_t j = 0; j< N; j++){
            double edge = (cluster_boxes[i].pose - cluster_boxes[j].pose).norm();
            if(edge<edge_dis_th){
                vertex_edges.emplace_back(std::make_pair(cluster_boxes[j].label,edge));
                AdjacencyMatrix(i,j) = 1;
                EdgeMatrix(i,j) = edge;
                if(j>=i+1){                                          // only count once
                    frame_graph.edges.emplace_back(std::make_pair(i,j));
                    frame_graph.edge_value.emplace_back(edge);
                    frame_graph.edge_weights.emplace_back((edge_dis_th-edge)/edge_dis_th); //[0,edge_dis_th]->[1,0]
                }
            }
        }

        // build vertes desc: main for loop clousre detection
        for(size_t m=0;m<vertex_edges.size();m++){
            if(vertex_edges[m].first == 1){ // x - vehicle
                NodeEmbeddings_Local(i,int(vertex_edges[m].second/sub_interval_value))++;
            }
            else if(vertex_edges[m].first == 2){ // x - truck
                NodeEmbeddings_Local(i,subinterval+int(vertex_edges[m].second/sub_interval_value))++;
            }
            else if(vertex_edges[m].first == 3){ // x - pole
                NodeEmbeddings_Local(i,subinterval*2+int(vertex_edges[m].second/sub_interval_value))++;
            }
        }

    }

    if(frame_graph.node_labels.size()==0) return  frame_graph;
    // only 0.0xms -> 0.1ms 
    Eigen::MatrixXd NodeEmbeddings_Global= MatrixDecomposing(AdjacencyMatrix,graph_node_dimension);
    NodeEmbeddings_Local = NodeEmbeddings_Local.array().colwise()/NodeEmbeddings_Local.rowwise().norm().array();
    NodeEmbeddings_Global = NodeEmbeddings_Global.array().colwise()/NodeEmbeddings_Global.rowwise().sum().array();
    NodeEmbeddings.leftCols(subinterval*3) = NodeEmbeddings_Local;
    NodeEmbeddings.rightCols(graph_node_dimension) = NodeEmbeddings_Global;

    for(size_t i=0;i<N;i++){
        Eigen::MatrixXd evec_sort_row = NodeEmbeddings.row(i);
        std::vector<float> node_desf(evec_sort_row.data(),evec_sort_row.data()+evec_sort_row.size());
        frame_graph.node_desc.emplace_back(node_desf);
    }
    

    // build local sub triangle for outlier pruning
    for(size_t i=0;i<N;i++){
        frame_graph.node_sub_triangles.emplace_back(std::vector<Eigen::Vector3d>());
        std::vector<int> indices;
        std::vector<Eigen::Vector3d> nodeSubgraphTriangle;
        auto edge_list = EdgeMatrix.row(i);
        for(int m=0;m<edge_list.size();m++){
            if(edge_list[m]<subgraph_edge_th && edge_list[m]!=0) indices.push_back(m);
        }

        if(indices.size()>2) {
            for(size_t m=0; m<indices.size();m++){
                if(m==(indices.size()-1)) break;
                for(size_t n=m+1; n<indices.size();n++){
                    std::vector<float> sub_triangle(3);
                    sub_triangle[0] = (float)(frame_graph.node_centers[i]-frame_graph.node_centers[indices[m]]).norm();
                    sub_triangle[1] = (float)(frame_graph.node_centers[i]-frame_graph.node_centers[indices[n]]).norm();
                    sub_triangle[2] = (float)(frame_graph.node_centers[indices[m]]-frame_graph.node_centers[indices[n]]).norm();
                    std::sort(sub_triangle.begin(), sub_triangle.end()); // sort triangle edges
                    nodeSubgraphTriangle.emplace_back(Eigen::Vector3d(sub_triangle[0],sub_triangle[1],sub_triangle[2]));
                }
            }
        }
        frame_graph.node_sub_triangles[i].assign(nodeSubgraphTriangle.begin(),nodeSubgraphTriangle.end());
    }

    frame_graph.edge_matrix = EdgeMatrix; // for subsequent correspondences pruning

    return frame_graph;
}



/*
    Decomposing adjacency matrix to get node vector
*/
Eigen::MatrixXd MatrixDecomposing(Eigen::MatrixXd MatrixInput,int Dimension){ 

        // decomposing adjacency matrix
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;
        es.compute(MatrixInput);
        Eigen::MatrixXcd evecs = es.eigenvectors();            
        Eigen::MatrixXd evecs_abs = evecs.real().cwiseAbs();   // get abs eigen vector

        Eigen::MatrixXcd evals = es.eigenvalues();             
        Eigen::MatrixXd abs_evals = evals.real().cwiseAbs();

        // sort to get max->min eigen value
        std::vector<float> vec_evals(&abs_evals(0, 0),abs_evals.data()+abs_evals.size()); // Eigen::MatrixXf --> std::vector
        std::vector<int> indices(vec_evals.size());
        std::iota(indices.begin(), indices.end(), 0);  // sort: get d eigen vector corresponding to the d largest eigen value
        std::sort(indices.begin(), indices.end(), [&vec_evals](int i, int j) { return vec_evals[i] > vec_evals[j]; });

        // get node vector
        Eigen::MatrixXd evecs_sort = Eigen::MatrixXd::Zero(vec_evals.size(),Dimension);
        size_t iter = std::min(Dimension,static_cast<int>(indices.size()));
        for(size_t i=0;i<iter;i++){
            evecs_sort.col(i) = evecs_abs.col(indices[i]);
        }

        return evecs_sort;
    }

}

