# 
#    SemGraphSLAM is heavily inspired by the framework of KISS-ICP (https://github.com/PRBonn/kiss-icp).
#    We are deeply appreciative of Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss for their contributions to the open-source community.
#
# This file is covered by the LICENSE file in the root of this project.
# contact: Neng Wang, <neng.wang@hotmail.com>

cmake_minimum_required(VERSION 3.16...3.26)

project(semgraph_slam)

set(ignore ${CATKIN_INSTALL_INTO_PREFIX_ROOT})
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/semgraph_slam/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/semgraph_slam ${CMAKE_CURRENT_BINARY_DIR}/semgraph_slam)
else()
   message(STATUS "semgraph_slam not exist")
endif()


if("$ENV{ROS_VERSION}" STREQUAL "1")
  message(STATUS "semgraph_slam-ROS1 wrapper will be compiled")
  find_package(
    catkin REQUIRED
    COMPONENTS geometry_msgs
               nav_msgs
               sensor_msgs
               roscpp
               rosbag
               std_msgs
               tf
               tf2
               tf2_ros)
  catkin_package()

  find_package(PCL REQUIRED)
  find_package(GTSAM REQUIRED)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
  )

  add_executable(semSLAM ros1/semSLAM.cpp)
  target_include_directories(semSLAM PUBLIC include ${catkin_INCLUDE_DIRS})
  target_link_libraries(semSLAM graph_slam::pipeline ${catkin_LIBRARIES} gtsam)
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  message(STATUS "semgraph_slam-ROS2 wrapper will be compiled")
  find_package(ament_cmake REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(visualization_msgs REQUIRED) 

  find_package(PCL REQUIRED)
  find_package(GTSAM REQUIRED)

  include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
  )
  
  add_executable(semSLAM ros2/semSLAM.cpp)
  target_include_directories(semSLAM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(semSLAM graph_slam::pipeline ${catkin_LIBRARIES} gtsam)

  install(TARGETS semSLAM RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}/)
  
  ament_target_dependencies(
    semSLAM
    rcutils
    rclcpp
    rclcpp_components
    nav_msgs
    sensor_msgs
    tf2_ros
    visualization_msgs)
  ament_package()
else()
  message(FATAL_ERROR "catkin or colcon not found SG-SLAM-ROS disabled")

endif()



