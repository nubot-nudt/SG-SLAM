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

//ROS2
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"




namespace semgraph_slam_ros {

// Odometry constructor (front-end)
class Odometry : public rclcpp::Node  {
public:

    Odometry();
    ~Odometry(){};
    void RegisterFrame();
    std::string lidar_path_;
    std::string label_path_;
    int cloudInd;
private:

    int queue_size_{100};
    
    /// Tools for broadcasting TFs.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Data subscribers.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    // Data publishers.
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr kpoints_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_graph_map_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_graph_edge_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_graph_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr relocalization_corr_publisher_;

    // Path publisher
    nav_msgs::msg::Path path_msg_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_publisher_;


    std::vector<geometry_msgs::msg::Pose> ground_truth_poses_vec_;
    std::vector<geometry_msgs::msg::Pose> no_relocalization_poses_vec_;

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
class Mapping : public rclcpp::Node {
    public:
        Mapping();
        ~Mapping(){};
        void ScanMapping();
        void PublishMap(bool seq_finsih_flag);
        

    private:
        graph_slam::SemGraphMapping semGraphMappinger;
        graph_slam::SemGraphMappingConfig config_;

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
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pgo_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_graph_map_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pc_map_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr corres_lins_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_graph_edge_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_pc_publisher_;
};


}   // namespace graph_slam_ros
