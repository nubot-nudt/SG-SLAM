#This file is covered by the LICENSE file in the root of this project.

add_library(core STATIC)
add_library(graph_slam::core ALIAS core)
target_sources(core PRIVATE nanoflann.hpp CommonUtils.cpp)
target_link_libraries(core PUBLIC Eigen3::Eigen tsl::robin_map)
#OpenMP::OpenMP_CXX
set_global_target_properties(core)
