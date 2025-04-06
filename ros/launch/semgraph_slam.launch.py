from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

class Basic_config():
    def __init__(self):
        self.bagfile: str  = ""
        self.visualize: str  = "true"
        self.odom_frame: str = "odom"
        self.map_frame: str = "map"
        self.child_frame: str = "base_link"
        self.dataset: str = "kitti"

        self.current_pkg_path = get_package_share_directory("semgraph_slam")

        os.makedirs(os.path.join(self.current_pkg_path, "save"), exist_ok=True)
        os.makedirs(os.path.join(self.current_pkg_path, "data"), exist_ok=True)

        self.result_path: str = os.path.join(self.current_pkg_path, "save/kitti_odometry_00.txt")
        self.pgo_result_path: str = os.path.join(self.current_pkg_path, "save/kitti_slam_00.txt")
        self.graph_map_path: str = os.path.join(self.current_pkg_path, "data/graph_map_00.txt")
        self.graph_edge_path: str = os.path.join(self.current_pkg_path, "data/graph_edge_00.txt")


        self.lidar_path: str = "/home/wangneng/DataFast/kitti/sequences/00/velodyne/"
        self.label_path: str = "/home/wangneng/DataFast/kitti/sequences/00/labels/"

        # KISS-ICP params 
        self.deskew : bool = False
        self.relocalization_enable: bool = False
        self.loop_closure_enable: bool = True
        self.max_range: float = 100.0
        self.min_range: float = 5.0
        self.voxel_size: float = 1.0    
        self.max_points_per_voxel: int = 20
        self.initial_threshold: float = 2.0
        self.min_motion_th: float = 0.1

        # cluster params
        self.deltaA: float = 2.0
        self.deltaR: float = 0.5
        self.deltaP: float = 2.0
        
        # building graph params
        self.edge_dis_th: float = 60.0 
        self.subgraph_edge_th: float = 20.0
        self.subinterval: int = 30
        self.graph_node_dimension: int = 30
        self.nearest_neighbor_vehicle_disth: float = 2.0
        self.nearest_neighbor_pole_disth: float = 2.0
        self.max_local_graph_map_range: float = 100.0

        # relozalization params
        self.model_deviation_trans: float = 0.12
        self.model_deviation_rot: float = 0.01
        self.inlier_rate_th: float = 0.43

        # loop closure params
        self.global_des_dim: int = 231
        self.loop_candidate: int = 5
        self.keyframe_interval: int = 5
        self.max_distance_for_loop: float = 0.1
        self.graph_sim_th: float = 0.5
        self.back_sim_th: float = 0.58
        self.map_voxel_size_loop: float = 1.0
        self.frame_acc_pgo: int = 20
        


def generate_launch_description():
    current_pkg = FindPackageShare("semgraph_slam")
    config = Basic_config()

    sg_slam_node = Node(
        package="semgraph_slam",
        executable="semSLAM",
        name="semSLAM",
        output="screen",
        parameters=[
            {
                "odom_frame": config.odom_frame,
                "map_frame": config.map_frame,
                "child_frame": config.child_frame,
                "dataset": config.dataset,
                "result_path": config.result_path,
                "pgo_result_path": config.pgo_result_path,
                "graph_map_path": config.graph_map_path,
                "graph_edge_path": config.graph_edge_path,
                "lidar_path": config.lidar_path,
                "label_path": config.label_path,
                "relocalization_enable": config.relocalization_enable,
                "loop_closure_enable": config.loop_closure_enable,
                "max_range": config.max_range,
                "min_range": config.min_range,
                "deskew": config.deskew,
                "voxel_size": config.voxel_size,
                "max_points_per_voxel": config.max_points_per_voxel,
                "initial_threshold": config.initial_threshold,
                "min_motion_th": config.min_motion_th,
                "deltaA": config.deltaA,
                "deltaR": config.deltaR,
                "deltaP": config.deltaP,
                "edge_dis_th": config.edge_dis_th,
                "subgraph_edge_th": config.subgraph_edge_th,
                "subinterval": config.subinterval,
                "graph_node_dimension": config.graph_node_dimension,
                "nearest_neighbor_vehicle_disth": config.nearest_neighbor_vehicle_disth,
                "nearest_neighbor_pole_disth": config.nearest_neighbor_pole_disth,
                "max_local_graph_map_range": config.max_local_graph_map_range,
                "model_deviation_trans": config.model_deviation_trans,
                "model_deviation_rot": config.model_deviation_rot,
                "inlier_rate_th": config.inlier_rate_th,
                "global_des_dim": config.global_des_dim,
                "loop_candidate": config.loop_candidate,
                "keyframe_interval": config.keyframe_interval,
                "max_distance_for_loop": config.max_distance_for_loop,
                "graph_sim_th": config.graph_sim_th,
                "back_sim_th": config.back_sim_th,
                "map_voxel_size_loop": config.map_voxel_size_loop,
                "frame_acc_pgo": config.frame_acc_pgo,
            }
        ],
    )

    # RVIZ2
    rviz2_node = Node(
                    package="rviz2",
                    executable="rviz2",
                    output={"both": "log"},
                    arguments=["-d", PathJoinSubstitution([current_pkg, "rviz", "sg_slam_ros2.rviz"])],
                    condition=IfCondition(config.visualize),
    )
    
     # ROS2 bag play
    bag_play = ExecuteProcess(
                    cmd=["ros2", "bag", "play", config.bagfile],
                    output="screen",
                    condition=IfCondition(
                        PythonExpression(["'", config.bagfile, "' != ''"])
                    ),
    )
    
    return LaunchDescription(
        [
            sg_slam_node,
            rviz2_node,
            bag_play,
        ]
    )
