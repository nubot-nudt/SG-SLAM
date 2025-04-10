/* 
    SemGraphSLAM is heavily inspired by the framework of KISS-ICP (https://github.com/PRBonn/kiss-icp).
    We are deeply appreciative of Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss for their contributions to the open-source community.
*/

// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#include <Eigen/Core>
#include <vector>
#include <fstream>

// SG-SLAM
#include "semSLAM.hpp"
#include "InsUtils.hpp"


// ROS2 headers
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// data transmission type for front-end thread -> back-end thread
struct scan_data
{
    int cloud_id;
    graph_slam::V3d_i frame;
    Sophus::SE3d poses;
    graph_slam::Graph graph;

};
std::queue<scan_data> scan_queue;
std::mutex scan_mutex; // mutex for scan_queue

bool seq_finish = false; // flag to indicate end of sequence

std::vector<Sophus::SE3d> poses_pgo_vec_; // poses for PGO
int recent_update_idx=0;      // index of the most recent update in PGO

namespace semgraph_slam_ros {

// Front-end thread constructor initialization
Odometry::Odometry()
    : rclcpp::Node("odometry_node")  {
     
    dataset_ = declare_parameter<std::string>("dataset",dataset_); // dataset name: kitti, mulran, apollo
    child_frame_ = declare_parameter<std::string>("child_frame",child_frame_); // child frame name, i.e., velodyne frame
    odom_frame_ = declare_parameter<std::string>("odom_frame",odom_frame_);// odom frame name
    result_path_ = declare_parameter<std::string>("result_path",result_path_);// path to save odometry result, kitti format
    config_.max_range = declare_parameter<double>("max_range", config_.max_range); // max range of point cloud, preprocessing
    config_.min_range = declare_parameter<double>("min_range", config_.min_range); // min range of point cloud, preprocessing
    config_.deskew = declare_parameter<bool>("deskew", config_.deskew);  // the flag to deskew the point cloud, preprocessing
    config_.relocalization_enable = declare_parameter<bool>("relocalization_enable", config_.relocalization_enable);  // the flag to enable relocalization
    config_.voxel_size = declare_parameter<double>("voxel_size", config_.max_range / 100.0); // voxel size for downsampling, preprocessing, i.e., v in the paper
    config_.max_points_per_voxel = declare_parameter<int>("max_points_per_voxel", config_.max_points_per_voxel); // the maximum number of points per voxel, local mapping, i.e., N_max in the paper
    config_.initial_threshold = declare_parameter<double>("initial_threshold", config_.initial_threshold);// the initial threshold for initial pose estimation, kiss-icp parameter
    config_.min_motion_th = declare_parameter<double>("min_motion_th", config_.min_motion_th);// the minimum motion threshold for pose estimation, kiss-icp parameter
    
    config_.model_deviation_trans = declare_parameter<double>("model_deviation_trans", config_.model_deviation_trans); // the distance threshold for performing relocalization, i.e., t_o in the paper
    config_.model_deviation_rot = declare_parameter<double>("model_deviation_rot", config_.model_deviation_rot);  // the angle threshold for performing relocalization, i.e., r_o in the paper
    config_.inlier_rate_th = declare_parameter<double>("inlier_rate_th", config_.inlier_rate_th);  // the inlier ratio threshold for relocalization, i.e., I_r in the paper
 
    config_.deltaA = declare_parameter<double>("deltaA", config_.deltaA);  // cluster parameter for building graph: Z-axis angle resolution
    config_.deltaR = declare_parameter<double>("deltaR", config_.deltaR);  // cluster parameter for building graph: XY-plane radius
    config_.deltaP = declare_parameter<double>("deltaP", config_.deltaP);  // cluster parameter for building graph: Resolution of the angle with the positive x-axis direction
    config_.edge_dis_th = declare_parameter<double>("edge_dis_th", config_.edge_dis_th); // the edge threshold for building graph, i.e., d_max in the SGLC paper
    config_.subinterval = declare_parameter<int>("subinterval", config_.subinterval); // the interval for graph descriptors, i.e., d_max/d_i in the SGLC paper
    config_.subgraph_edge_th = declare_parameter<double>("subgraph_edge_th", config_.subgraph_edge_th); // the edge threshold for outlier pruning
    config_.graph_node_dimension = declare_parameter<int>("graph_node_dimension", config_.graph_node_dimension); // the dimension of the node descriptors,

    config_.nearest_neighbor_vehicle_disth = declare_parameter<double>("nearest_neighbor_vehicle_disth", config_.nearest_neighbor_vehicle_disth); // the distance threshold for NMS of vehicle in local graph map
    config_.nearest_neighbor_pole_disth = declare_parameter<double>("nearest_neighbor_pole_disth", config_.nearest_neighbor_pole_disth); // the distance threshold for NMS of pole in local graph map
    config_.max_local_graph_map_range = declare_parameter<double>("max_local_graph_map_range", config_.max_local_graph_map_range); // the max range of local graph map, i.e., d_max in the paper
    
    lidar_path_ = declare_parameter<std::string>("lidar_path",lidar_path_);  // path to the point cloud data,
    label_path_ = declare_parameter<std::string>("label_path",label_path_);  // path to the label data,


    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(this->get_logger(),"[WARNING] max_range is smaller than min_range, setting min_range to 0.0");
        config_.min_range = 0.0;
    }

    cloudInd = 0;

    fout_odom_.open(result_path_,std::ofstream::trunc); // odometry result file, kitti format
    fout_odom_.close(); // clear the file

    // Construct the main KISS-ICP odometry node
    semGraphSlamer = graph_slam::SemGraphSLAM(config_);

    // Initialize publishers
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", qos);// odometry publisher
    frame_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("frame", qos); // current frame publisher
    kpoints_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("keypoints", qos);  // keypoints publisher
    local_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_map", qos); //local point cloud map publisher
    local_graph_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("local_graph_map", qos); // local graph map publisher
    local_graph_edge_publisher_ = create_publisher<visualization_msgs::msg::Marker>("local_graph_edge", qos); // local graph map edge publisher
    frame_graph_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("frame_graph", qos); // current frame graph publisher
    relocalization_corr_publisher_ = create_publisher<visualization_msgs::msg::Marker>("relocalization_corr", qos); // relocalization node correspondences publisher

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize trajectory publisher
    path_msg_.header.frame_id = odom_frame_;
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("trajectory", qos);

    // Broadcast a static transformation that links with identity the specified base link to the
    // pointcloud_frame, basically to always be able to visualize the frame in rviz
    if (child_frame_ != "base_link") {
        static auto br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
        geometry_msgs::msg::TransformStamped alias_transform_msg;
        alias_transform_msg.header.stamp = this->get_clock()->now();
        alias_transform_msg.transform.translation.x = 0.0;
        alias_transform_msg.transform.translation.y = 0.0;
        alias_transform_msg.transform.translation.z = 0.0;
        alias_transform_msg.transform.rotation.x = 0.0;
        alias_transform_msg.transform.rotation.y = 0.0;
        alias_transform_msg.transform.rotation.z = 0.0;
        alias_transform_msg.transform.rotation.w = 1.0;
        alias_transform_msg.header.frame_id = child_frame_;
        alias_transform_msg.child_frame_id = "base_link";
        br->sendTransform(alias_transform_msg);
    }

    // Publish odometry msg
    RCLCPP_INFO(this->get_logger(), "SG-SLAM: ROS2 Odometry Node Initialized");
}
float total_time = 0;
int frame_count = 0;

/*
    Front-end thread main function
*/
void Odometry::RegisterFrame() {
    rclcpp::Rate loop(10);  // 10 Hz
    


    std::vector<double> times_vec; // timestamp for scan deskewing, only for mulran dataset
    if(dataset_ == "mulran"){ // mulran dataset deskewing from kissicp
        int H = 64;
        int W = 1024;
        std::vector<double> result(H * W);
        for (int i = 0; i < H * W; ++i) {
            double pos = static_cast<double>(std::floor(i / static_cast<double>(H))) / W;
            result[i] = pos;
        }
        times_vec.assign(result.begin(), result.end());
    }

    // Main loop
    while(rclcpp::ok()){

        // Load data: scan and label
        std::stringstream lidar_data_path;
        std::stringstream lidar_label_path;
        lidar_data_path << lidar_path_ << std::setfill('0') << std::setw(6)
                    << cloudInd << ".bin";
        lidar_label_path << label_path_ << std::setfill('0') << std::setw(6)
                    << cloudInd << ".label";

        auto frame_raw= loadCloud(lidar_data_path.str(), lidar_label_path.str());
        if (frame_raw.first.empty()) {
            RCLCPP_WARN(this->get_logger(),"No more data, shutting down");
            seq_finish = true;  // a flag to indicate end of sequence, for back-end thread
            break; 
        }

        // Time recorder for each frame
        auto start_time = std::chrono::steady_clock::now(); 
        frame_count = frame_count + 1;

        if(dataset_ == "mulran"){
            if(frame_raw.first.size() != times_vec.size()) times_vec = std::vector<double>(frame_raw.first.size(), 1);
        }

        // Main process for odometry
        const auto &[frame, keypoints, graph] = semGraphSlamer.mainProcess(frame_raw.first,frame_raw.second,times_vec,dataset_); 
        const auto pose = semGraphSlamer.poses().back(); // the estimated current pose

        // Convert from Eigen to ROS types
        const Eigen::Vector3d t_current = pose.translation(); 
        const Eigen::Quaterniond q_current = pose.unit_quaternion();
        
        // Push data to queue for back-end thread
        scan_mutex.lock();
        scan_queue.push({cloudInd, frame, pose, graph}); 
        scan_mutex.unlock();

        // Get the duration of each frame
        auto end_time = std::chrono::steady_clock::now(); 
        auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count(); 
        total_time = total_time + duration;
        std::cout<<CYAN<<"[ Odometry ]: time:"<<duration<<",average time:"<<total_time/frame_count<<",frame count:"<<frame_count<<std::endl;
        
        // Publish Broadcast the tf
        std_msgs::msg::Header velodyne_header;
        velodyne_header.frame_id = 'velodyne';
        velodyne_header.stamp = rclcpp::Clock().now();

        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = velodyne_header.stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = child_frame_;
        transform_msg.transform.rotation.x = q_current.x();
        transform_msg.transform.rotation.y = q_current.y();
        transform_msg.transform.rotation.z = q_current.z();
        transform_msg.transform.rotation.w = q_current.w();
        transform_msg.transform.translation.x = t_current.x();
        transform_msg.transform.translation.y = t_current.y();
        transform_msg.transform.translation.z = t_current.z();
        tf_broadcaster_->sendTransform(transform_msg);

        // Publish odometry msg
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = velodyne_header.stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = child_frame_;
        odom_msg.pose.pose.orientation.x = q_current.x();
        odom_msg.pose.pose.orientation.y = q_current.y();
        odom_msg.pose.pose.orientation.z = q_current.z();
        odom_msg.pose.pose.orientation.w = q_current.w();
        odom_msg.pose.pose.position.x = t_current.x();
        odom_msg.pose.pose.position.y = t_current.y();
        odom_msg.pose.pose.position.z = t_current.z();
        odom_publisher_->publish(odom_msg);

        // Save odometry result, kitti format
        Eigen::Quaterniond q_eigen(q_current.w(),q_current.x(),q_current.y(),q_current.z());
        q_eigen.normalize();
        Eigen::Matrix3d rot_mat = q_eigen.toRotationMatrix();
        Eigen::Matrix4d poseMat = pose.matrix();
        fout_odom_.open(result_path_,std::ofstream::app);
        fout_odom_.setf(std::ios::fixed, std::ios::floatfield);
        fout_odom_.precision(16);
        fout_odom_ << poseMat(0,0) << " " << poseMat(0,1) << " " << poseMat(0,2) << " " << poseMat(0,3) << " "
                << poseMat(1,0) << " " << poseMat(1,1) << " " << poseMat(1,2) << " " << poseMat(1,3) << " "
                << poseMat(2,0) << " " << poseMat(2,1) << " " << poseMat(2,2) << " " << poseMat(2,3) << std::endl;
        fout_odom_.close();


        // Publish SG-SLAM internal data, just for debugging and viualization
        // publish trajectory msg
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.pose = odom_msg.pose.pose;
        pose_msg.header = odom_msg.header;
        path_msg_.poses.push_back(pose_msg);
        traj_publisher_->publish(path_msg_);
        
        // Transform frame to slam coordination, just for viualization
        Sophus::SE3d odom_pose_prev = semGraphSlamer.poses()[recent_update_idx];
        Sophus::SE3d pgo_pose_prev = poses_pgo_vec_.empty()? Sophus::SE3d(): poses_pgo_vec_[recent_update_idx];
        Sophus::SE3d pose_diff = pose.inverse() * (pgo_pose_prev * (odom_pose_prev.inverse() * pose));

        std::pair<std::vector<Eigen::Vector3d>,std::vector<int>> new_frame;
        for(size_t i=0;i<frame.first.size();i++){
            new_frame.first.push_back(pose_diff*frame.first[i]);
            new_frame.second.push_back(frame.second[i]);
        }
        
        // Publish current frame
        std_msgs::msg::Header frame_header = velodyne_header;
        frame_header.frame_id = child_frame_;
        frame_publisher_->publish(convertPointCloudSemanticMSG(new_frame.first,new_frame.second, frame_header));

        // Publish local point cloud map
        std_msgs::msg::Header local_map_header = velodyne_header;
        local_map_header.frame_id = odom_frame_;
        local_map_publisher_->publish(convertPointCloudSemanticRGBMSG(semGraphSlamer.LocalMap(), local_map_header));

        // Publish local graph map
        auto graph_map_msg = converGrapp2MSG(semGraphSlamer.LocalGraphMap().node_centers,semGraphSlamer.LocalGraphMap().node_labels,semGraphSlamer.LocalGraphMap().edges,local_map_header);
        local_graph_map_publisher_->publish(graph_map_msg.first);
        local_graph_edge_publisher_->publish(graph_map_msg.second);

        // Publish relocalization results
        auto new_frame_graph = graph;
        auto initial_poses_for_relo = semGraphSlamer.initial_guess_for_relocalization;
        for(int i=0;i<new_frame_graph.node_centers.size();i++){
            new_frame_graph.node_centers[i] = initial_poses_for_relo*new_frame_graph.node_centers[i];
            new_frame_graph.node_centers[i][2] = new_frame_graph.node_centers[i][2] + 30; // for viualization
        }
        auto frame_graph_msg = converGrapp2MSG(new_frame_graph.node_centers,new_frame_graph.node_labels,new_frame_graph.edges,local_map_header);
        frame_graph_publisher_->publish(frame_graph_msg.first);

        auto relo_corr = semGraphSlamer.relocalization_corr;
        for(size_t i=0; i<relo_corr.first.size();i++){
            relo_corr.first[i] = initial_poses_for_relo*relo_corr.first[i];
            relo_corr.first[i][2] = relo_corr.first[i][2] + 30; // for viualization
        }
        relocalization_corr_publisher_->publish(convertRelocalizationCorr2MSG(relo_corr, local_map_header));

        cloudInd++;
        loop.sleep();
        rclcpp::spin_some(shared_from_this());

    }
}


// Back-end thread constructor initialization
Mapping::Mapping()
    : rclcpp::Node("mapping_node")  {

    map_frame_ = declare_parameter<std::string>("map_frame", map_frame_);// the map frame
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);// the odometry frame
    pgo_result_path_ = declare_parameter<std::string>("pgo_result_path", pgo_result_path_); // the path of pgo poses result file, kitti format
    graph_map_path_ = declare_parameter<std::string>("graph_map_path", graph_map_path_); // the path of saving global graph map file
    graph_edge_path_ = declare_parameter<std::string>("graph_edge_path", graph_edge_path_); // the path of saving global graph edge file
    child_frame_ = declare_parameter<std::string>("child_frame", child_frame_); // the child frame
    config_.global_des_dim = declare_parameter<int>("global_des_dim", config_.global_des_dim);  // the global descriptor dimension
    config_.loop_candidate = declare_parameter<int>("loop_candidate", config_.loop_candidate);  // the number of candidate loop closures
    config_.edge_dis_th = declare_parameter<double>("edge_dis_th", config_.edge_dis_th);           // the edge distance for building graph
    config_.subinterval = declare_parameter<int>("subinterval", config_.subinterval);           // the interval for graph descriptors
    config_.keyframe_interval = declare_parameter<int>("keyframe_interval", config_.keyframe_interval);      // the interval for keyframe generation, i.e., n in the paper
    config_.search_results_num = declare_parameter<int>("search_results_num", config_.search_results_num);   // the number of search results for loop closure
    config_.max_distance_for_loop = declare_parameter<float>("max_distance_for_loop", config_.max_distance_for_loop);  // the descriptor distance for geometry verification
    config_.graph_sim_th = declare_parameter<double>("graph_sim_th", config_.graph_sim_th);        // the similarity threshold for graph verification
    config_.back_sim_th = declare_parameter<double>("back_sim_th", config_.back_sim_th);           // the similarity threshold for bakcground verification
    config_.map_voxel_size_loop = declare_parameter<double>("map_voxel_size_loop", config_.map_voxel_size_loop); // the voxel size for estimating loop transformation
    config_.frame_acc_pgo = declare_parameter<int>("frame_acc_pgo", config_.frame_acc_pgo);     // the frame accuracy for performing pgo
    config_.loop_closure_enable = declare_parameter<bool>("loop_closure_enable", config_.loop_closure_enable);  // enable loop closure detection


    // Construct the main mapping node
    semGraphMappinger = graph_slam::SemGraphMapping(config_);

    rclcpp::QoS qos(rclcpp::KeepLast{queue_size_});
    traj_pgo_publisher_ = create_publisher<nav_msgs::msg::Path>("trajectory_pgo", qos);
    global_graph_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("global_graph_map", qos);
    global_pc_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("global_pc_map", qos);
    corres_lins_publisher_ = create_publisher<visualization_msgs::msg::Marker>("node_correspondences", qos);
    global_graph_edge_publisher_ = create_publisher<visualization_msgs::msg::Marker>("graph_edges", qos);
    loop_pc_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("loop_pc", qos);
 

    // Resulut file
    fout_pgo_result_.open(pgo_result_path_,std::ofstream::trunc);
    fout_pgo_result_.close(); // clear the file

    fout_global_graph_map_.open(graph_map_path_,std::ofstream::trunc);
    fout_global_graph_map_.close(); // clear the file

    fout_global_graph_edge_.open(graph_edge_path_,std::ofstream::trunc);
    fout_global_graph_edge_.close(); // clear the file

    // Construct gtsam graph (PGO)
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    
    double loopNoiseScore = 0.1;
    gtsam::Vector robustNoiseVector6(6);
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
                        loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

    // Publish odometry msg
    RCLCPP_INFO(this->get_logger(), "SG-SLAM: ROS2 Mapping Node Initialized");
}

/*
    Back-end thread main function
*/
void Mapping::ScanMapping(){
    int run_isam_count = 0;            // the count for running isam
    int run_map_update_count = 0;      // the count for running map update
    bool process_flag_debug = false;   
    bool seq_finsih_mapping = false;   // the flag for sequences finishing
    bool map_update_flag = false;      // the flag for map update
    int frame = 0;
    while(rclcpp::ok()&&config_.loop_closure_enable){   // loop clousure enable
        
        // Timer
        auto start_time = std::chrono::steady_clock::now();

        process_flag_debug = false;

        if(seq_finish) seq_finsih_mapping = true;  // sequence finishing flag from front-end

        // Check is empty?
        if(!scan_queue.empty()){

            // Get frame data
            scan_mutex.lock();
            const auto frame_data = scan_queue.front();
            scan_queue.pop();
            scan_mutex.unlock();

            process_flag_debug = true;

            // Frame count
            run_isam_count++;
            run_map_update_count++;

            // Get the current pose
            Eigen::Matrix3d poses_R = frame_data.poses.rotationMatrix();
            Eigen::Vector3d poses_t = frame_data.poses.translation();
            int cloudInd = frame_data.cloud_id;
            poses_vec_.emplace_back(frame_data.poses);
            poses_pgo_vec_.emplace_back(frame_data.poses);
            
            // Main process for back-end thread
            semGraphMappinger.mainProcess(frame_data.cloud_id, frame_data.graph);

            // Only keyframe for PGO
            if(semGraphMappinger.is_keyframe_vec[frame_data.cloud_id]){

                // True loop closure
                if(semGraphMappinger.loop_flag){
                    const auto &loop_pair = semGraphMappinger.loop_pair_vec.back();  // the loop pair (current keyframe, loop keyframe)
                    auto &loop_trans = semGraphMappinger.loop_trans_vec.back();      // the corresponding loop transformation
                    std::cout <<WHITE<< "[  Mapping ]: <Loop Detection> between:" << loop_pair.first<< " ------ " <<loop_pair.second << \
                                        ", Time [gen des]:"<<semGraphMappinger.time_gen_des<<"ms, [loop search]:"<<semGraphMappinger.time_search<<"ms"<<std::endl;
                    std::cout <<WHITE<< "[  Mapping ]: <Loop transform>: " << loop_trans << std::endl;

                    // Add loop factor
                    gtsam::Point3 ttem(loop_trans.block<3,1>(0,3));
                    gtsam::Rot3 Rtem(loop_trans.block<3,3>(0,0));
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(loop_pair.second, loop_pair.first, gtsam::Pose3(Rtem, ttem), robustLoopNoise));

                    // Add loop closure point cloud visualization
                    std::vector<Eigen::Vector4d> loop_frame_pc;
                    auto loop_frame_pose = poses_pgo_vec_[loop_pair.second];
                    int loop_key_frame_idx = semGraphMappinger.loop_frame_key_idx_vec.back();
                    const auto &loop_frame_graph = semGraphMappinger.keyframe_graph_vec_[loop_key_frame_idx];
                    std::vector<Eigen::Vector4d> frame_points_front(loop_frame_graph.front_points.first.size());
                    tbb::parallel_for(size_t(0),frame_points_front.size(), [&](size_t i){
                        frame_points_front[i].head<3>() =  loop_frame_pose*loop_frame_graph.front_points.first[i];
                        frame_points_front[i](3) = loop_frame_graph.front_points.second[i];
                    });

                    loop_frame_pc.insert(loop_frame_pc.end(),frame_points_front.begin(),frame_points_front.end());
                    std::vector<Eigen::Vector4d> frame_points_bk(loop_frame_graph.back_points.first.size());
                    tbb::parallel_for(size_t(0),frame_points_bk.size(), [&](size_t i){
                        frame_points_bk[i].head<3>() =  loop_frame_pose*loop_frame_graph.back_points.first[i];
                        frame_points_bk[i](3) = loop_frame_graph.back_points.second[i];
                    });
                    loop_frame_pc.insert(loop_frame_pc.end(),frame_points_bk.begin(),frame_points_bk.end());
                    
                    // Publish loop point cloud
                    std_msgs::msg::Header loop_frame_header;
                    loop_frame_header.frame_id = odom_frame_;
                    loop_frame_header.stamp = rclcpp::Clock().now();
                    sensor_msgs::msg::PointCloud2 loop_frame_pc_msg = convertPointCloudSemanticRGBMSG(loop_frame_pc, loop_frame_header);
                    loop_pc_publisher_->publish(loop_frame_pc_msg);
                }
                else{  // No loop closure
                    std::cout <<WHITE<< "[  Mapping ]: <No Loop>  :" << frame_data.cloud_id<< \
                                        ",  Time [gen des]:"<<semGraphMappinger.time_gen_des<<"ms"<<std::endl;
                    // Publish empty loop point cloud
                    std::vector<Eigen::Vector4d> loop_frame_pc;
                    loop_frame_pc.emplace_back((0,0,0,0));
                    std_msgs::msg::Header loop_frame_header;
                    loop_frame_header.frame_id = odom_frame_;
                    loop_frame_header.stamp = rclcpp::Clock().now();
                    sensor_msgs::msg::PointCloud2 loop_frame_pc_msg = convertPointCloudSemanticRGBMSG(loop_frame_pc, loop_frame_header);
                    loop_pc_publisher_->publish(loop_frame_pc_msg);
                }
            }
            
            // Add piror node in factor graph
            if(frame_data.cloud_id==0){
                gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0,gtsam::Pose3(gtsam::Rot3(poses_R), gtsam::Point3(poses_t)),odometryNoise));
                initial.insert(0,gtsam::Pose3(gtsam::Rot3(poses_R),gtsam::Point3(poses_t)));
            }
            else{
                // Add odometry factor
                Eigen::Vector3d t_ab = poses_vec_[cloudInd - 1].translation();
                Eigen::Matrix3d R_ab = poses_vec_[cloudInd - 1].rotationMatrix();

                t_ab = R_ab.transpose() * (poses_t- t_ab);
                R_ab = R_ab.transpose() * poses_R;

                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd, gtsam::Pose3(gtsam::Rot3(R_ab),gtsam::Point3(t_ab)), odometryNoise));
                initial.insert(cloudInd, gtsam::Pose3(gtsam::Rot3(poses_R),gtsam::Point3(poses_t)));
                    
            }
        }

        // Perform PGO
        if(run_isam_count >=config_.frame_acc_pgo || seq_finsih_mapping){
            run_isam_count = 0;

            // Factor graph optimization
            isam->update(gtSAMgraph,initial);
            isam->update();
            gtSAMgraph.resize(0);
            initial.clear();
            resultsIsam = isam->calculateEstimate();

            // Update poses
            std::cout<<WHITE<<"resultsIsam size:"<<resultsIsam.size()<<std::endl;
            recent_update_idx = resultsIsam.size() - 1;
            fout_pgo_result_.open(pgo_result_path_,std::ofstream::trunc);
            fout_pgo_result_.close(); // clear the file
            for(int node_idx=0;node_idx<int(resultsIsam.size());node_idx++){
                gtsam::Pose3 pose_pgo_scan = resultsIsam.at(node_idx).cast<gtsam::Pose3>();
                Sophus::SE3d pose_sophus_scan(pose_pgo_scan.rotation().matrix(), pose_pgo_scan.translation());
                poses_pgo_vec_[node_idx] = pose_sophus_scan;

                // Save PGO result
                if(seq_finsih_mapping){
                    fout_pgo_result_.open(pgo_result_path_,std::ofstream::app);
                    fout_pgo_result_.setf(std::ios::fixed, std::ios::floatfield);
                    fout_pgo_result_.precision(16);
                    Eigen::Matrix4d poseMat = pose_sophus_scan.matrix();
                    fout_pgo_result_ << poseMat(0,0) << " " << poseMat(0,1) << " " << poseMat(0,2) << " " << poseMat(0,3) << " "
                            << poseMat(1,0) << " " << poseMat(1,1) << " " << poseMat(1,2) << " " << poseMat(1,3) << " "
                            << poseMat(2,0) << " " << poseMat(2,1) << " " << poseMat(2,2) << " " << poseMat(2,3) << std::endl;
                    fout_pgo_result_.close();
                }
            }

            // Publish PGO poses msg
            nav_msgs::msg::Path path_pgo_;
            for(int node_idx=0;node_idx<int(resultsIsam.size());node_idx++){
                auto trans_t = poses_pgo_vec_[node_idx].translation();
                Eigen::Quaterniond trans_q(poses_pgo_vec_[node_idx].rotationMatrix());
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.frame_id = odom_frame_;
                pose_msg.header.stamp = rclcpp::Clock().now();
                pose_msg.pose.position.x = trans_t[0];
                pose_msg.pose.position.y = trans_t[1];
                pose_msg.pose.position.z = trans_t[2];
                pose_msg.pose.orientation.x = trans_q.x();
                pose_msg.pose.orientation.y = trans_q.y();
                pose_msg.pose.orientation.z = trans_q.z();
                pose_msg.pose.orientation.w = trans_q.w();
                path_pgo_.poses.push_back(pose_msg);
            }
            path_pgo_.header.frame_id = odom_frame_;
            path_pgo_.header.stamp = rclcpp::Clock().now();
            traj_pgo_publisher_->publish(path_pgo_);
        }

        // Publish global pc map & global graph map
        if(run_map_update_count>=(config_.frame_acc_pgo)||seq_finsih_mapping){
            run_map_update_count = 0;
            semGraphMappinger.UpdateMapping(poses_pgo_vec_,recent_update_idx);
            std::cout <<WHITE<< "[  Mapping ]: <Update Map> instance size:"<<semGraphMappinger.global_graph_map.size()<<std::endl;
            PublishMap(seq_finsih_mapping);
        }

        // Timer
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        if(process_flag_debug||seq_finsih_mapping){
            if(run_isam_count==0) std::cout<<WHITE<<"[  Mapping ]: <Run isam>, Time All:"<<duration<<"ms"<<std::endl;
            else std::cout<<WHITE<<"[  Mapping ]: Time All:"<<duration<<"ms"<<std::endl;
        }

        if(seq_finsih_mapping) break; // all frames have been processed

        rclcpp::spin_some(shared_from_this());
    }  
}

/*
    Publish the global pc map and global graph map
*/
void Mapping::PublishMap(bool seq_finsih_flag){

    std_msgs::msg::Header global_graph_header;
    global_graph_header.frame_id = odom_frame_;
    
    global_graph_header.stamp = rclcpp::Clock().now();
    sensor_msgs::msg::PointCloud2 global_graph_map_msg = convertPointCloudSemanticRGBMSG(semGraphMappinger.global_graph_map, global_graph_header);
    global_graph_map_publisher_->publish(global_graph_map_msg);

    visualization_msgs::msg::Marker graph_edge_marker = convertEdgeMsg(semGraphMappinger.global_graph_map, semGraphMappinger.global_graph_edge, global_graph_header);
    global_graph_edge_publisher_->publish(graph_edge_marker);

    // Publish global pc map after all frames have been processed
    if(seq_finsih_flag){
        int down_meters = 0.0;
        rclcpp::Rate rate(100); 
        for(size_t i=0; i<semGraphMappinger.keyframe_idx_vec_.size(); i++){
            
            std::vector<Eigen::Vector4d> global_pc_map;
            int idx = semGraphMappinger.keyframe_idx_vec_[i]; // idx in all frame
            const auto &graph = semGraphMappinger.keyframe_graph_vec_[i];
            if(idx>recent_update_idx) break;

            const auto &pose = poses_pgo_vec_[idx]; // pose of current frame

            std::vector<Eigen::Vector4d> frame_points_front(graph.front_points.first.size());
            tbb::parallel_for(size_t(0),frame_points_front.size(), [&](size_t i){
                frame_points_front[i].head<3>() =  pose*graph.front_points.first[i];
                frame_points_front[i](2) = frame_points_front[i](2)-down_meters;
                frame_points_front[i](3) = graph.front_points.second[i];
            });

            global_pc_map.insert(global_pc_map.end(),frame_points_front.begin(),frame_points_front.end());
            std::vector<Eigen::Vector4d> frame_points_bk(graph.back_points.first.size());
            tbb::parallel_for(size_t(0),frame_points_bk.size(), [&](size_t i){
                frame_points_bk[i].head<3>() =  pose*graph.back_points.first[i];
                frame_points_bk[i](2) = frame_points_bk[i](2)-down_meters;
                frame_points_bk[i](3) = graph.back_points.second[i];
            });
            global_pc_map.insert(global_pc_map.end(),frame_points_bk.begin(),frame_points_bk.end());

            sensor_msgs::msg::PointCloud2 global_pc_map_msg = convertPointCloudSemanticRGBMSG(global_pc_map, global_graph_header);
            global_pc_map_publisher_->publish(global_pc_map_msg);
            rate.sleep();
        }

        // Publish edge
        visualization_msgs::msg::Marker marker;
        std::vector<geometry_msgs::msg::Point> points;
        marker.ns = "lines";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.4; 
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.9;
        for(size_t i=0;i<semGraphMappinger.global_graph_map.size();i++){
            geometry_msgs::msg::Point p1, p2;
            p1.x = p2.x = semGraphMappinger.global_graph_map[i][0];
            p1.y = p2.y = semGraphMappinger.global_graph_map[i][1];
            p1.z = semGraphMappinger.global_graph_map[i][2];
            p2.z = semGraphMappinger.global_graph_map[i][2] - down_meters;
            points.push_back(p1);
            points.push_back(p2);
        }
        marker.points = points;
        marker.header = global_graph_header;
        corres_lins_publisher_->publish(marker);

        //save graph map
        fout_global_graph_map_.open(graph_map_path_,std::ofstream::app);
        for(size_t i=0;i<semGraphMappinger.global_graph_map.size();i++){
            fout_global_graph_map_<<semGraphMappinger.global_graph_map[i][0]<<" "<<semGraphMappinger.global_graph_map[i][1]<<" "<<semGraphMappinger.global_graph_map[i][2]<<" "<<
                        semGraphMappinger.global_graph_map[i][3]<<std::endl;
        }

        //save graph edge
        fout_global_graph_edge_.open(graph_edge_path_,std::ofstream::app);
        for(size_t i=0;i<semGraphMappinger.global_graph_edge.size();i++){
            fout_global_graph_edge_<<semGraphMappinger.global_graph_edge[i].first<<" "<<semGraphMappinger.global_graph_edge[i].second<<std::endl;
        }
    }
    
}

}   // namespace graph_slam_ros


/*
    Main function for SG-SLAM, a dual-thread framework, including odometry and mapping
*/
int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    rclcpp::Rate init_wait(0.5);

    auto odom_node = std::make_shared<semgraph_slam_ros::Odometry>();
    auto mapping_node = std::make_shared<semgraph_slam_ros::Mapping>();

    int cloudInd = 0;
    std::cout<<"waiting for initialization"<<std::endl;
    init_wait.sleep();

    std::thread odometry_thread([&odom_node]() { odom_node->RegisterFrame();}); // front-end thread, including preprocessing, odometry and relocalization 
    std::thread mapping_thread([&mapping_node]() { mapping_node->ScanMapping();}); // back-end thread, including loop closing, PGO and mapping

    odometry_thread.join();
    mapping_thread.join();
    
    return 0;
}
