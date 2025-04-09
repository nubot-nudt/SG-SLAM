#This file is covered by the LICENSE file in the root of this project.

add_library(loopclosure STATIC)
add_library(graph_slam::loopclosure ALIAS loopclosure)
target_sources(loopclosure PRIVATE  LoopClosure.cpp Hungarian.cpp
                            FastIcp.cpp PlaneIcp.cpp )
target_link_libraries(loopclosure PUBLIC Eigen3::Eigen tsl::robin_map TBB::tbb Sophus::Sophus Ceres::ceres graph_slam::core)
#OpenMP::OpenMP_CXX
set_global_target_properties(loopclosure)
