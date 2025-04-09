#This file is covered by the LICENSE file in the root of this project.

add_library(frontend STATIC)
add_library(graph_slam::frontend ALIAS frontend)
target_sources(frontend PRIVATE Registration.cpp Deskew.cpp VoxelHashMap.cpp 
                            Preprocessing.cpp Threshold.cpp 
                            SemanticCluster.cpp SemGraph.cpp GraphMap.cpp)
target_link_libraries(frontend PUBLIC Eigen3::Eigen tsl::robin_map TBB::tbb Sophus::Sophus Ceres::ceres)
#OpenMP::OpenMP_CXX
set_global_target_properties(frontend)
