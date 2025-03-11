/* 
    SemGraphSLAM is heavily inspired by the framework of KISS-ICP (https://github.com/PRBonn/kiss-icp).
    We are deeply appreciative of Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss for their contributions to the open-source community.
*/

// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#pragma once

#include <thread>
#include <mutex>
#include <queue>
#include <chrono>

// gtsam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// sg-slam
#include "semgraph_slam/pipeline/SemGraphSLAM.hpp"
#include "semgraph_slam/pipeline/SemGraphMapping.hpp"

// ROS
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"




namespace semgraph_slam_ros {

// Odometry constructor (front-end)
class Odometry {
public:

    Odometry(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
    ~Odometry(){};
    void RegisterFrame();
    std::string lidar_path_;
    std::string label_path_;
    int cloudInd;
private:

    // Ros node stuff
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    int queue_size_{100};
    
    // Tools for broadcasting TFs.
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Data subscribers.
    ros::Subscriber pointcloud_sub_;

    // Data publishers.
    ros::Publisher odom_publisher_;
    ros::Publisher traj_publisher_;
    nav_msgs::Path path_msg_;
    ros::Publisher frame_publisher_;
    ros::Publisher kpoints_publisher_;
    ros::Publisher local_map_publisher_;
    ros::Publisher local_graph_map_publisher_;
    ros::Publisher local_graph_edge_publisher_;

    ros::Publisher frame_graph_publisher_;
    ros::Publisher relocalization_corr_publisher_;

    std::vector<geometry_msgs::Pose> ground_truth_poses_vec_;
    std::vector<geometry_msgs::Pose> no_relocalization_poses_vec_;

    graph_slam::SemGraphSLAM semGraphSlamer;
    graph_slam::SemGraphSLAMConfig config_;

    // Global/map coordinate frame.
    std::string odom_frame_{"odom"};
    std::string child_frame_{"base_link"};
    std::string result_path_{"./"};
    std::ofstream fout_odom_;
    std::string dataset_;
};

// Mapping constructor (back-end)
class Mapping{
    public:
        Mapping(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
        ~Mapping(){};
        void ScanMapping();
        void PublishMap(bool seq_finsih_flag);
        

    private:
        graph_slam::SemGraphMapping semGraphMappinger;
        graph_slam::SemGraphMappingConfig config_;

        // Ros node stuff
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        int queue_size_{1000};
        
        // Coordinate frame stuff
        std::string map_frame_{"map"};
        std::string odom_frame_{"odom"};
        std::string child_frame_{"base_link"};
        std::string pgo_result_path_{"./"};
        std::string graph_map_path_{"./"};
        std::string graph_edge_path_{"./"};

        // Data save
        std::vector<Sophus::SE3d> poses_vec_;
        std::ofstream fout_pgo_result_;
        std::ofstream fout_global_graph_map_;
        std::ofstream fout_global_graph_edge_;

        // Gtsam stuff
        gtsam::Values initial;
        gtsam::NonlinearFactorGraph gtSAMgraph;
        gtsam::ISAM2 *isam;
        gtsam::Values resultsIsam;
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;  
        gtsam::noiseModel::Base::shared_ptr robustLoopNoise; 
        
        // Data publishers
        ros::Publisher traj_pgo_publisher_;
        ros::Publisher global_graph_map_publisher_;
        ros::Publisher global_pc_map_publisher_;
        ros::Publisher corres_lins_publisher_;
        ros::Publisher global_graph_edge_publisher_;
        ros::Publisher loop_pc_publisher_;
};


}   // namespace graph_slam_ros
