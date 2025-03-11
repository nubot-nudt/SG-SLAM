// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>
// reference: CVC [https://github.com/wangx1996/Lidar-Segementation]


#include "SemanticCluster.hpp"

namespace graph_slam{

double Polar_angle_cal(double x, double y){
	double temp_tangle = 0;
	if(x== 0 && y ==0){
	     temp_tangle = 0;
	}else if(y>=0){
	     temp_tangle = atan2(y,x);
	}else if(y<0){
	     temp_tangle = atan2(y,x) + 2*M_PI;
	}
	return temp_tangle;
}

std::vector<PointAPR> SemanticCluster::calculateAPR(const std::vector<Eigen::Vector3d>& cloud_IN){
    std::vector<PointAPR> vapr_;
    for (size_t i =0; i<cloud_IN.size(); ++i){
        PointAPR par;
        par.polar_angle = Polar_angle_cal(cloud_IN[i][0],cloud_IN[i][1]);
        par.range = sqrt(cloud_IN[i][0]*cloud_IN[i][0] + cloud_IN[i][1]*cloud_IN[i][1]); 
        par.azimuth = atan2(cloud_IN[i][2], par.range); 
        if(par.range < min_range_){
                min_range_ = par.range;
        }
        if(par.range > max_range_){
                max_range_ = par.range;
        }
        vapr_.push_back(par);
    }
    length_ = int((max_range_ - min_range_)/deltaR_)+1; 
    width_  = int(round(360/deltaP_)); 
    height_ = int(((max_azimuth_ - min_azimuth_)*180/M_PI)/deltaA_)+1;

    return vapr_;
}

std::unordered_map<int, Voxel_C> SemanticCluster::build_hash_table(const std::vector<PointAPR>& vapr){
    std::unordered_map<int, Voxel_C> map_out;
     std::vector<int> ri;
     std::vector<int> pi;
     std::vector<int> ai;
     for(int i =0; i< (int)vapr.size(); ++i){
           int azimuth_index = int(((vapr[i].azimuth-min_azimuth_)*180/M_PI)/deltaA_);
           int polar_index = int(vapr[i].polar_angle*180/M_PI/deltaP_);
           int range_index = int((vapr[i].range - min_range_)/deltaR_);
	   
           int voxel_index = (polar_index*(length_)+range_index)+azimuth_index*(length_)*(width_);
           std::unordered_map<int, Voxel_C>::iterator it_find;
           it_find = map_out.find(voxel_index);
           if (it_find != map_out.end()){
                it_find->second.index.push_back(i);

           }else{
                Voxel_C vox;
                vox.haspoint =true;
                vox.index.push_back(i); 
                vox.index.swap(vox.index);
                map_out.insert(std::make_pair(voxel_index,vox));
           }

    }
    return map_out;
}

void SemanticCluster::find_neighbors(int polar,
                                     int range,
                                     int azimuth,
                                     std::vector<int>& neighborindex) {
    for (int z = azimuth - 1; z <= azimuth + 1; z++) {
           if (z < 0 || z > (height_ - 1)) {
                continue;
           }

           for (int y = range - 1; y <= range + 1; y++) {
                if (y < 0 || y > (length_ - 1)) {
                    continue;
                }

                for (int x = polar - 1; x <= polar + 1; x++) {
                    int px = x;
                    if (x < 0) {
                        px = width_ - 1;
                    }
                    if (x > (width_ - 1)) {
                        px = 0;
                    }
                    neighborindex.push_back((px * (length_) + y) +
                                            z * (length_) * (width_));
                }
           }
    }
}

void SemanticCluster::mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2) {
    for (size_t i = 0; i < cluster_indices.size(); i++) {
           if (cluster_indices[i] == idx1) {
                cluster_indices[i] = idx2;
           }
    }
}

std::vector<int> SemanticCluster::cluster(std::unordered_map<int, Voxel_C>& map_in, const std::vector<PointAPR>& vapr) {
    int current_cluster = 0;
    std::vector<int> cluster_indices = std::vector<int>(vapr.size(), -1);
    for (size_t i = 0; i < vapr.size(); ++i) {
           if (cluster_indices[i] != -1)
                continue;
           int azimuth_index =
               int((vapr[i].azimuth - min_azimuth_) * 180 / M_PI / deltaA_);
           int polar_index = int(vapr[i].polar_angle * 180 / M_PI / deltaP_);
           int range_index = int((vapr[i].range - min_range_) / deltaR_);
           int voxel_index = (polar_index * (length_) + range_index) +
                             azimuth_index * (length_) * (width_);

           std::unordered_map<int, Voxel_C>::iterator it_find;
           std::unordered_map<int, Voxel_C>::iterator it_find2;

           it_find = map_in.find(voxel_index);
           std::vector<int> neightbors;

           if (it_find != map_in.end()) {
                std::vector<int> neighborid;
                find_neighbors(polar_index, range_index, azimuth_index,
                               neighborid);
                for (size_t k = 0; k < neighborid.size(); ++k) {
                    it_find2 = map_in.find(neighborid[k]);

                    if (it_find2 != map_in.end()) {
                        for (size_t j = 0; j < it_find2->second.index.size();
                             ++j) {
                            neightbors.push_back(it_find2->second.index[j]);
                        }
                    }
                }
           }

           neightbors.swap(neightbors);

           if (neightbors.size() > 0) {
                for (size_t j = 0; j < neightbors.size(); ++j) {
                    int oc = cluster_indices[i];
                    int nc = cluster_indices[neightbors[j]];
                    if (oc != -1 && nc != -1) {
                        if (oc != nc)
                            mergeClusters(cluster_indices, oc, nc);
                    } else {
                        if (nc != -1) {
                            cluster_indices[i] = nc;
                        } else {
                            if (oc != -1) {
                                cluster_indices[neightbors[j]] = oc;
                            }
                        }
                    }
                }
           }

           if (cluster_indices[i] == -1) {
                current_cluster++;
                cluster_indices[i] = current_cluster;
                for (size_t s = 0; s < neightbors.size(); ++s) {
                    cluster_indices[neightbors[s]] = current_cluster;
                }
           }
    }
    return cluster_indices;
}

bool compare_cluster(std::pair<int,int> a,std::pair<int,int> b){
    return a.second>b.second;
}//upper sort

std::vector<int> SemanticCluster::remove_noise(std::vector<int> values) {
    std::vector<int> cluster_index;
    std::unordered_map<int, int> histcounts;
    for (size_t i = 0; i < values.size(); i++) {
           if (histcounts.find(values[i]) == histcounts.end()) {
                histcounts[values[i]] = 1;
           } else {
                histcounts[values[i]] += 1;
           }
    }
    // int max = 0, maxi;
    std::vector<std::pair<int, int>> tr(histcounts.begin(), histcounts.end());
    sort(tr.begin(), tr.end(), compare_cluster);
    for (size_t i = 0; i < tr.size(); ++i) {
           if (tr[i].second > 10) {  // 如果小于10个点，就不聚类
                cluster_index.push_back(tr[i].first);
           }
    }

    return cluster_index;
}

/*
    Main cluster function
*/
std::vector<Bbox> ClusterPoints(const std::vector<Eigen::Vector3d>& frame,
                          const std::vector<int>& frame_label,
                          V3d_i& background_points,
                          V3d_i& ClusterSemPoints,
                          double delta_azimuth,
                          double delta_range,
                          double delta_polar) {

    // auto start_time = std::chrono::steady_clock::now();  
    std::vector<std::vector<Eigen::Vector3d>*> pc_semantic;
    
    // 260 is number of semantic labels in semantickitti
    for (size_t i = 0; i < 26; i++) {
           pc_semantic.emplace_back(new std::vector<Eigen::Vector3d>());
    }
    
    // label mapped
    for (size_t i = 0; i < frame.size(); i++) {
        // object 
           if (frame_label[i] == 1 ||     // car
               frame_label[i] == 4 ||     // truck
               frame_label[i] == 5 ||     // other_vehicle
               frame_label[i] == 16 ||    // trunk
               frame_label[i] == 18 ) {    //pole
                pc_semantic[frame_label[i]]->emplace_back(frame[i]);
                ClusterSemPoints.first.emplace_back(frame[i]);
                ClusterSemPoints.second.emplace_back(frame_label[i]);
           }
           //back ground
           if(  
                frame_label[i]==14 ||
                frame_label[i]==15 ||
                frame_label[i]==9 || 
                frame_label[i]==13){// build or road 

                background_points.first.emplace_back(frame[i]);
                background_points.second.emplace_back(frame_label[i]);
           }

    }

    std::vector<double> param(3, 0);
    param[0] = delta_azimuth;
    param[1] = delta_range;
    param[2] = delta_polar;

    // Cluster
    std::vector<Bbox> frame_bbox_cluster;
    for (size_t i = 0; i < 26; i++) {
           if (pc_semantic[i]->empty())
                continue;
            
            if(i==13){  // special param for building
                    param[0] = 5;
                    param[1] = 2;
                    param[2] = 5;
            }

           SemanticCluster pc_cluster(param);
           const auto& capr = pc_cluster.calculateAPR(*pc_semantic[i]);
           std::unordered_map<int, Voxel_C> hash_table;
           hash_table = pc_cluster.build_hash_table(capr);
           const auto& cluster_indices = pc_cluster.cluster(hash_table, capr);

           // get each clusters' box
           struct label_box {
                int num;
                double dimension[6];
           };
           std::unordered_map<int, label_box> histcounts;

            std::vector<int> cluster_id; // for view

           for (size_t j = 0; j < cluster_indices.size(); j++) {
                if (histcounts.find(cluster_indices[j]) == histcounts.end()) {
                    histcounts[cluster_indices[j]].dimension[0] = (*pc_semantic[i])[j][0];  // init min x
                    histcounts[cluster_indices[j]].dimension[1] = (*pc_semantic[i])[j][1];  // init min y
                    histcounts[cluster_indices[j]].dimension[2] = (*pc_semantic[i])[j][2];  // init min z
                    histcounts[cluster_indices[j]].dimension[3] = (*pc_semantic[i])[j][0];  // init max x
                    histcounts[cluster_indices[j]].dimension[4] = (*pc_semantic[i])[j][1];  // init max y
                    histcounts[cluster_indices[j]].dimension[5] = (*pc_semantic[i])[j][2];  // init max z
                    histcounts[cluster_indices[j]].num = 1;
                } else {
                    histcounts[cluster_indices[j]].num += 1;
                    if ((*pc_semantic[i])[j][0] < histcounts[cluster_indices[j]].dimension[0])
                        histcounts[cluster_indices[j]].dimension[0] = (*pc_semantic[i])[j][0];
                    else if ((*pc_semantic[i])[j][0] > histcounts[cluster_indices[j]].dimension[3])
                        histcounts[cluster_indices[j]].dimension[3] = (*pc_semantic[i])[j][0];

                    if ((*pc_semantic[i])[j][1] < histcounts[cluster_indices[j]].dimension[1])
                        histcounts[cluster_indices[j]].dimension[1] = (*pc_semantic[i])[j][1];
                    else if ((*pc_semantic[i])[j][1] > histcounts[cluster_indices[j]].dimension[4])
                        histcounts[cluster_indices[j]].dimension[4] = (*pc_semantic[i])[j][1];

                    if ((*pc_semantic[i])[j][2] < histcounts[cluster_indices[j]].dimension[2])
                        histcounts[cluster_indices[j]].dimension[2] = (*pc_semantic[i])[j][2];
                    else if ((*pc_semantic[i])[j][2] > histcounts[cluster_indices[j]].dimension[5])
                        histcounts[cluster_indices[j]].dimension[5] = (*pc_semantic[i])[j][2];
                }
           }
           std::vector<std::pair<int, label_box>> tr(histcounts.begin(),
                                                     histcounts.end());
           for (size_t m = 0; m < tr.size(); m++) {
                if (tr[m].second.num < 10)
                    continue;
                Bbox bbox_cluster;
                 if(i==1||i==4||i==5){
                    // if(tr[m].second.num < 30) continue;
                    bbox_cluster.label = 1; // vechicle
                }
                else if(i==16){
                    // if(tr[m].second.num < 20) continue;
                    bbox_cluster.label = 2; // trunk
                }
                else if(i==18){
                    // if(tr[m].second.num < 10) continue;
                    bbox_cluster.label = 3; // pole
                }          
                bbox_cluster.center[0] =
                    (tr[m].second.dimension[0] + tr[m].second.dimension[3]) / 2;
                bbox_cluster.center[1] =
                    (tr[m].second.dimension[1] + tr[m].second.dimension[4]) / 2;
                bbox_cluster.center[2] =
                    (tr[m].second.dimension[2] + tr[m].second.dimension[5]) / 2;
                bbox_cluster.dimension[0] =
                    tr[m].second.dimension[3] - tr[m].second.dimension[0];
                bbox_cluster.dimension[1] =
                    tr[m].second.dimension[4] - tr[m].second.dimension[1];
                bbox_cluster.dimension[2] =
                    tr[m].second.dimension[5] - tr[m].second.dimension[2];

                bbox_cluster.points_num = tr[m].second.num;
                if(bbox_cluster.label ==1){
                    if(bbox_cluster.dimension[0]<0.5 || bbox_cluster.dimension[1]<0.5 || bbox_cluster.dimension[2]<0.3 )
                        continue;
                }
                else if (bbox_cluster.label == 2 || bbox_cluster.label == 3)
                {
                    if(bbox_cluster.dimension[2]<0.3){
                        continue;
                    }
                }
                frame_bbox_cluster.emplace_back(bbox_cluster);
           }
    }

    // dealloc memory
    for (size_t i = 0; i < pc_semantic.size(); i++) {
            delete pc_semantic[i];
    }
    pc_semantic.clear();
    // auto end_time = std::chrono::steady_clock::now();
    // auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    // std::cout<<"cluster: time consumption:"<<duration<<" ms"<<std::endl;
    return frame_bbox_cluster;
}

}
