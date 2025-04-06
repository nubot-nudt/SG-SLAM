/* 
    SemGraphSLAM is heavily inspired by the framework of KISS-ICP (https://github.com/PRBonn/kiss-icp).
    We are deeply appreciative of Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss for their contributions to the open-source community.
*/

// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <regex>
#include <string>
#include <unordered_map>
#include <omp.h>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS2
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Color codes for terminal output
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


// Label mapping, Note: mulran mapping all vehicle label -> 0
std::unordered_map<int, int> label_map={
  {0, 0},     // "unlabeled"
  {1, 0},     // "outlier" mapped to "unlabeled" --------------------------mapped
  {10, 1},     //! "car"  1
  {11, 2},     // "bicycle"
  {13, 5},     // 5 "bus" mapped to "other-vehicle" --------------------------mapped
  {15, 3},     // "motorcycle"
  {16, 5},     // 5"on-rails" mapped to "other-vehicle" ---------------------mapped
  {18, 4},     // 4"truck"
  {20, 5},     // 5"other-vehicle"
  {30, 6},     // "person"
  {31, 7},     // "bicyclist"
  {32, 8},     // "motorcyclist"
  {40, 9},     // "road"
  {44, 10},    // "parking"
  {48, 11},    // "sidewalk"
  {49, 12},    // "other-ground"
  {50, 13},    // "building"
  {51, 14},    // "fence"
  {52, 0},     // "other-structure" mapped to "unlabeled" ------------------mapped
  {60, 9},     // "lane-marking" to "road" ---------------------------------mapped
  {70, 15},    // "vegetation"
  {71, 16},    // "trunk"
  {72, 17},    // "terrain"
  {80, 18},    // "pole"
  {81, 19},    // "traffic-sign"
  {99, 0},     // "other-object" to "unlabeled" ----------------------------mapped
  {252, 20},    // "moving-car"
  {253, 21},    // "moving-bicyclist"
  {254, 22},    // "moving-person"
  {255, 23},    // "moving-motorcyclist"
  {256, 24},    // "moving-on-rails" mapped to "moving-other-vehicle" ------mapped
  {257, 24},    // "moving-bus" mapped to "moving-other-vehicle" -----------mapped
  {258, 25},    // "moving-truck"
  {259, 24}    // "moving-other-vehicle"
};

// Color map for visualization
std::unordered_map<int,Eigen::Vector3i> color_map={
			{0,{0,0,0}},
			{0,{255,0,0}},
			{1,{100,150,245}},
			{2,{100,230,245}},
			{5,{100,80,250}},
			{3,{30,60,150}},
			{5,{0,0,255}},
			{4,{80,30,180}},
			{5,{0,0,255}},
			{6,{255,30,30}},
			{7,{255,40,200}},
			{8,{150,30,90}},
			{9,{255,0,255}},
			{10,{255,150,255}},
			{11,{75,0,75}},
			{12,{175,0,75}},
			{13,{255,200,0}},
			{14,{255,120,50}},
			{0,{255,150,0}},
			{9,{150,255,170}},
			{15,{0,175,0}},
			{16,{135,60,0}},
			{17,{150,240,80}},
			{18,{255,240,150}},
			{19,{255,0,0}},
			{0,{50,255,255}},
			{20,{100,150,245}},
			{21,{255,40,200}},
			{22,{255,30,30}},
			{23,{150,30,90}},
			{24,{0,0,255}},
			{24,{100,80,250}},
			{25,{80,30,180}},
			{24,{255,0,0}}
};



namespace semgraph_slam_ros {
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using Header = std_msgs::msg::Header;

std::string FixFrameId(const std::string &frame_id) {
    return std::regex_replace(frame_id, std::regex("^/"), "");
}

auto GetTimestampField(const PointCloud2 &msg) {
    PointField timestamp_field;
    for (const auto &field : msg.fields) {
        if ((field.name == "t" || field.name == "timestamp" || field.name == "time")) {
            timestamp_field = field;
        }
    }
    if (!timestamp_field.count) {
        throw std::runtime_error("Field 't', 'timestamp', or 'time'  does not exist");
    }
    return timestamp_field;
}

// Normalize timestamps from 0.0 to 1.0
auto NormalizeTimestamps(const std::vector<double> &timestamps) {
    const double max_timestamp = *std::max_element(timestamps.cbegin(), timestamps.cend());
    // check if already normalized
    if (max_timestamp < 1.0) return timestamps;
    std::vector<double> timestamps_normalized(timestamps.size());
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
                   [&](const auto &timestamp) { return timestamp / max_timestamp; });
    return timestamps_normalized;
}

auto ExtractTimestampsFromMsg(const PointCloud2 &msg, const PointField &field) {
    // Extract timestamps from cloud_msg
    const size_t n_points = msg.height * msg.width;
    std::vector<double> timestamps;
    timestamps.reserve(n_points);

    // Option 1: Timestamps are unsigned integers -> epoch time.
    if (field.name == "t" || field.name == "timestamp") {
        sensor_msgs::PointCloud2ConstIterator<uint32_t> msg_t(msg, field.name);
        for (size_t i = 0; i < n_points; ++i, ++msg_t) {
            timestamps.emplace_back(static_cast<double>(*msg_t));
        }
        // Covert to normalized time, between 0.0 and 1.0
        return NormalizeTimestamps(timestamps);
    }

    // Option 2: Timestamps are floating point values between 0.0 and 1.0
    // field.name == "timestamp"
    sensor_msgs::PointCloud2ConstIterator<double> msg_t(msg, field.name);
    for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        timestamps.emplace_back(*msg_t);
    }
    return timestamps;
}

auto CreatePointCloud2Msg(const size_t n_points, const Header &header, bool timestamp = false) {
    PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    cloud_msg.header = header;
    cloud_msg.header.frame_id = FixFrameId(cloud_msg.header.frame_id);
    cloud_msg.fields.clear();
    int offset = 0;
    offset = addPointField(cloud_msg, "x", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "y", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "z", 1, PointField::FLOAT32, offset);
    offset += sizeOfPointField(PointField::FLOAT32);
    if (timestamp) {
        // asuming timestamp on a velodyne fashion for now (between 0.0 and 1.0)
        offset = addPointField(cloud_msg, "time", 1, PointField::FLOAT64, offset);
        offset += sizeOfPointField(PointField::FLOAT64);
    }

    // Resize the point cloud accordingly
    cloud_msg.point_step = offset;
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
    cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);
    modifier.resize(n_points);
    return cloud_msg;
}

auto CreatePointCloud2MsgSemantic(const size_t n_points, const Header &header, bool timestamp = false) {
    PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    cloud_msg.header = header;
    cloud_msg.header.frame_id = FixFrameId(cloud_msg.header.frame_id);
    cloud_msg.fields.clear();
    int offset = 0;
    offset = addPointField(cloud_msg, "x", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "y", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "z", 1, PointField::FLOAT32, offset);
    offset = addPointField(cloud_msg, "label",1,PointField::UINT16,offset);
    offset += sizeOfPointField(PointField::FLOAT32);
    if (timestamp) {
        // asuming timestamp on a velodyne fashion for now (between 0.0 and 1.0)
        offset = addPointField(cloud_msg, "time", 1, PointField::FLOAT64, offset);
        offset += sizeOfPointField(PointField::FLOAT64);
    }

    // Resize the point cloud accordingly
    cloud_msg.point_step = offset;
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
    cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);
    modifier.resize(n_points);
    return cloud_msg;
}

void FillPointCloud2XYZ(const std::vector<Eigen::Vector3d> &points, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z) {
        const Eigen::Vector3d &point = points[i];
        *msg_x = point.x();
        *msg_y = point.y();
        *msg_z = point.z();
    }
}
void FillPointCloud2XYZLabel(const std::vector<Eigen::Vector3d> &points,const std::vector<int> label, PointCloud2 &msg) {
    assert(points.size() == label.size());
    sensor_msgs::PointCloud2Iterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> msg_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<uint16_t> msg_label(msg, "label");
    for (size_t i = 0; i < points.size(); i++, ++msg_x, ++msg_y, ++msg_z,++msg_label) {
        const Eigen::Vector3d &point = points[i];
        *msg_x = point.x();
        *msg_y = point.y();
        *msg_z = point.z();
        *msg_label = label[i];
    }
}

void FillPointCloud2Timestamp(const std::vector<double> &timestamps, PointCloud2 &msg) {
    sensor_msgs::PointCloud2Iterator<double> msg_t(msg, "time");
    for (size_t i = 0; i < timestamps.size(); i++, ++msg_t) *msg_t = timestamps[i];
}

std::vector<double> GetTimestamps(const PointCloud2 &msg) {
    auto timestamp_field = GetTimestampField(msg);

    // Extract timestamps from cloud_msg
    std::vector<double> timestamps = ExtractTimestampsFromMsg(msg, timestamp_field);

    return timestamps;
}

std::pair<std::vector<Eigen::Vector3d>,std::vector<int>> PointCloud2ToEigen(const PointCloud2 &msg) {
    std::vector<Eigen::Vector3d> points;
    std::vector<int> semantic_label;
    points.reserve(msg.height * msg.width);
    sensor_msgs::PointCloud2ConstIterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(msg, "z");
    bool has_label = false;
    for(const auto& field: msg.fields) { if(field.name == "label") {has_label=true; break;}}
    if(has_label==true){
        sensor_msgs::PointCloud2ConstIterator<uint16_t> msg_label(msg,"label");
        for (size_t i = 0; i < msg.height * msg.width; ++i, ++msg_x, ++msg_y, ++msg_z, ++msg_label) {
            points.emplace_back(*msg_x, *msg_y, *msg_z);
            semantic_label.emplace_back(*msg_label);
        }
    }
    else{
            for (size_t i = 0; i < msg.height * msg.width; ++i, ++msg_x, ++msg_y, ++msg_z) {
            points.emplace_back(*msg_x, *msg_y, *msg_z);
            semantic_label.emplace_back(-1);
        }
        std::cout<<"Label not exist"<<std::endl;
    }
    return std::make_pair(points,semantic_label);
}


PointCloud2 EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points, const Header &header) {
    PointCloud2 msg = CreatePointCloud2Msg(points.size(), header);
    FillPointCloud2XYZ(points, msg);
    return msg;
}

std::pair<PointCloud2,visualization_msgs::msg::Marker> converGrapp2MSG(const std::vector<Eigen::Vector3d> &nodes, 
                                                                  const std::vector<int> &labels, 
                                                                  const std::vector<std::pair<int,int>> &edges, 
                                                                  const Header &header){
    // nodes
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc_in(new pcl::PointCloud<pcl::PointXYZRGBL>());
    for(size_t i=0;i<nodes.size();i++){
        int label   = labels[i];
        if(label==1) label = 1;   // vehicle
        else if(label==2) label = 16; // trunk
        else if(label==3) label = 18; // pole
        pcl::PointXYZRGBL pointLabel(color_map[label][0],color_map[label][1],color_map[label][2],label);;
        pointLabel.x = nodes[i].x();
        pointLabel.y = nodes[i].y();
        pointLabel.z = nodes[i].z();
        pc_in->push_back(pointLabel);
    }
    PointCloud2 PointsMsg;
    pcl::toROSMsg(*pc_in, PointsMsg);
    PointsMsg.header = header;
    PointsMsg.header.frame_id = FixFrameId(PointsMsg.header.frame_id);

    //edges
    visualization_msgs::msg::Marker marker;
    std::vector<geometry_msgs::msg::Point> points;
    marker.ns = "lines";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.2; 
    marker.color.r = 0.6;
    marker.color.g = 0.6;
    marker.color.b = 0.6;
    marker.color.a = 0.6;
    for(size_t i=0;i<edges.size();i++){
        geometry_msgs::msg::Point p1, p2;
        p1.x = nodes[edges[i].first][0];
        p1.y = nodes[edges[i].first][1];
        p1.z = nodes[edges[i].first][2];
        p2.x = nodes[edges[i].second][0];
        p2.y = nodes[edges[i].second][1];
        p2.z = nodes[edges[i].second][2];
        points.push_back(p1);
        points.push_back(p2);
    }

    marker.points = points;
    marker.header = header;
    marker.header.frame_id = FixFrameId(marker.header.frame_id);
    return std::make_pair(PointsMsg,marker);
}


visualization_msgs::msg::Marker convertRelocalizationCorr2MSG(const std::pair<std::vector<Eigen::Vector3d>,std::vector<Eigen::Vector3d>> &corr_nodes, const Header &header){

    visualization_msgs::msg::Marker marker;
    std::vector<geometry_msgs::msg::Point> points;
    marker.ns = "lines";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.5; 
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
    for(size_t i=0;i<corr_nodes.first.size();i++){
        geometry_msgs::msg::Point p1, p2;
        p1.x = corr_nodes.first[i].x();
        p1.y = corr_nodes.first[i].y();
        p1.z = corr_nodes.first[i].z();
        p2.x = corr_nodes.second[i].x();
        p2.y = corr_nodes.second[i].y();
        p2.z = corr_nodes.second[i].z();
        points.push_back(p1);
        points.push_back(p2);
    }

    marker.points = points;
    marker.header = header;
    marker.header.frame_id = FixFrameId(marker.header.frame_id);
    return marker;
}

PointCloud2 EigenToPointCloud2Semantic(const std::vector<Eigen::Vector3d> &points,const std::vector<int> &labels, const Header &header) {
    PointCloud2 msg = CreatePointCloud2MsgSemantic(points.size(), header);
    FillPointCloud2XYZLabel(points,labels, msg);
    return msg;
}

PointCloud2 convertPointCloudSemanticMSG(const std::vector<Eigen::Vector3d> &points,const std::vector<int> &labels, const Header &header){
    pcl::PointCloud<pcl::PointXYZL>::Ptr pc_in(new pcl::PointCloud<pcl::PointXYZL>());
    for(size_t i=0;i<points.size();i++){
        pcl::PointXYZL pointLabel;
        pointLabel.x = points[i].x();
        pointLabel.y = points[i].y();
        pointLabel.z = points[i].z();
        pointLabel.label = labels[i];
        pc_in->push_back(pointLabel);
    }
    PointCloud2 PointsMsg;
    pcl::toROSMsg(*pc_in, PointsMsg);
    PointsMsg.header = header;
    PointsMsg.header.frame_id = FixFrameId(PointsMsg.header.frame_id);
    return PointsMsg;
}

PointCloud2 convertPointCloudSemanticRGBMSG(const std::vector<Eigen::Vector4d> &points, const Header &header){
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc_in(new pcl::PointCloud<pcl::PointXYZRGBL>());
    for(size_t i=0;i<points.size();i++){
        int label   = points[i][3];
        if(label==1) label = 1;   // vehicle
        else if(label==2) label = 16; // trunk
        else if(label==3) label = 18; // pole
        if(label==0) continue;
        pcl::PointXYZRGBL pointLabel(color_map[label][0],color_map[label][1],color_map[label][2],label);;
        pointLabel.x = points[i].x();
        pointLabel.y = points[i].y();
        pointLabel.z = points[i].z();
        pc_in->push_back(pointLabel);
    }
    PointCloud2 PointsMsg;
    pcl::toROSMsg(*pc_in, PointsMsg);
    PointsMsg.header = header;
    PointsMsg.header.frame_id = FixFrameId(PointsMsg.header.frame_id);
    return PointsMsg;
}

visualization_msgs::msg::Marker convertEdgeMsg(const std::vector<Eigen::Vector4d> &graph_nodes,
                                            const std::vector<std::pair<int,int>> &graph_edges,
                                             const Header &header){
    visualization_msgs::msg::Marker marker;
    std::vector<geometry_msgs::msg::Point> points;
    marker.ns = "lines";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.2;
    marker.color.r = 0.6;
    marker.color.g = 0.6;
    marker.color.b = 0.6;
    marker.color.a = 0.6;
    for(size_t i=0;i<graph_edges.size();i++){
        geometry_msgs::msg::Point p1, p2;
        p1.x = graph_nodes[graph_edges[i].first][0];
        p1.y = graph_nodes[graph_edges[i].first][1];
        p1.z = graph_nodes[graph_edges[i].first][2];
        p2.x = graph_nodes[graph_edges[i].second][0];
        p2.y = graph_nodes[graph_edges[i].second][1];
        p2.z = graph_nodes[graph_edges[i].second][2];
        points.push_back(p1);
        points.push_back(p2);
    }

    marker.points = points;
    marker.header = header;
    marker.header.frame_id = FixFrameId(marker.header.frame_id);
    return marker;
}


PointCloud2 convertPointCloudSemanticRGBMSG_MultiThread(const std::vector<Eigen::Vector4d> &points, const Header &header){
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pc_in(new pcl::PointCloud<pcl::PointXYZRGBL>());
    pc_in->resize(points.size());
    for(size_t i=0;i<points.size();i++){
        int label   = points[i][3];
        if(label==1) label = 1;   // vehicle
        else if(label==2) label = 16; // trunk
        else if(label==3) label = 18; // pole
        pcl::PointXYZRGBL pointLabel(color_map[label][0],color_map[label][1],color_map[label][2],label);;
        pointLabel.x = points[i].x();
        pointLabel.y = points[i].y();
        pointLabel.z = points[i].z();
        pc_in->at(i) = pointLabel;
    }
    PointCloud2 PointsMsg;
    pcl::toROSMsg(*pc_in, PointsMsg);
    PointsMsg.header = header;
    PointsMsg.header.frame_id = FixFrameId(PointsMsg.header.frame_id);
    return PointsMsg;
}


PointCloud2 EigenToPointCloud2(const std::vector<Eigen::Vector3d> &points,
                               const std::vector<double> &timestamps,
                               const Header &header) {
    PointCloud2 msg = CreatePointCloud2Msg(points.size(), header, true);
    FillPointCloud2XYZ(points, msg);
    FillPointCloud2Timestamp(timestamps, msg);
    return msg;
}


std::pair<std::vector<Eigen::Vector3d>,std::vector<int>> loadCloud(std::string file_cloud, std::string file_label){
        std::ifstream in_label(file_label, std::ios::binary);
        if (!in_label.is_open()) {
            std::cerr << "No file:" << file_label << std::endl;
            // exit(-1);
            std::pair<std::vector<Eigen::Vector3d>,std::vector<int>> empty;
            return empty;
        }
        in_label.seekg(0, std::ios::end);
        uint32_t num_points = in_label.tellg() / sizeof(uint32_t);
        in_label.seekg(0, std::ios::beg);
        std::vector<uint32_t> values_label(num_points);
        in_label.read((char*)&values_label[0], num_points * sizeof(uint32_t));
        std::ifstream in_cloud(file_cloud, std::ios::binary);
        std::vector<float> values_cloud(4 * num_points);
        in_cloud.read((char*)&values_cloud[0], 4 * num_points * sizeof(float));
  

        std::vector<Eigen::Vector3d> pc_out(num_points);
        std::vector<int> label_out(num_points);

        for (uint32_t i = 0; i < num_points; ++i) {
            uint32_t sem_label;
            if (true) {
                sem_label = label_map[(int)(values_label[i] & 0x0000ffff)];
            } 
            else {
                sem_label = values_label[i];
            }
            pc_out[i] = Eigen::Vector3d(values_cloud[4 * i],values_cloud[4 * i + 1],values_cloud[4 * i + 2]);
            label_out[i] = (int) sem_label;
        }
        in_label.close();
        in_cloud.close();
        return std::make_pair(pc_out,label_out);
}

std::vector<geometry_msgs::msg::Pose> loadPose(std::string file_pose){
    std::vector<geometry_msgs::msg::Pose> poses_vec;
    std::ifstream f_pose(file_pose);
    std::istream_iterator<float> start(f_pose), end;
    std::vector<float> pose_temp(start, end);

    for(size_t i = 0; i < pose_temp.size(); i += 12){
        geometry_msgs::msg::Pose pose;
        pose.position.x = pose_temp[i+3];
        pose.position.y = pose_temp[i+7];
        pose.position.z = pose_temp[i+11];
        Eigen::Matrix3d rotation;
        rotation << pose_temp[i], pose_temp[i+1], pose_temp[i+2],
                    pose_temp[i+4], pose_temp[i+5], pose_temp[i+6],
                    pose_temp[i+8], pose_temp[i+9], pose_temp[i+10];
        Eigen::Quaterniond q(rotation);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        poses_vec.push_back(pose);
    }
    return poses_vec;
}

}   // namespace graph_slam_ros::utils
