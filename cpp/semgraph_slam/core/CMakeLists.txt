#This file is covered by the LICENSE file in the root of this project.

add_library(core STATIC)
add_library(graph_slam::core ALIAS core)
target_sources(core PRIVATE Registration.cpp Deskew.cpp VoxelHashMap.cpp 
                            Preprocessing.cpp Threshold.cpp 
                            SemanticCluster.cpp SemGraph.cpp
                            LoopClosure.cpp nanoflann.hpp Hungarian.cpp
                            FastIcp.cpp PlaneIcp.cpp GraphMap.cpp)
target_link_libraries(core PUBLIC Eigen3::Eigen tsl::robin_map TBB::tbb Sophus::Sophus Ceres::ceres)
#OpenMP::OpenMP_CXX
set_global_target_properties(core)
