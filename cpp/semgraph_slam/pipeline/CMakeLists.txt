#  SemGraphSLAM is heavily inspired by the framework of KISS-ICP (https://github.com/PRBonn/kiss-icp).
#  We are deeply appreciative of Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss for their contributions to the open-source community.
# 

# This file is covered by the LICENSE file in the root of this project.
# contact: Neng Wang, <neng.wang@hotmail.com>

add_library(pipeline STATIC)
add_library(graph_slam::pipeline ALIAS pipeline)
target_sources(pipeline PRIVATE SemGraphSLAM.cpp SemGraphMapping.cpp)
target_link_libraries(pipeline PUBLIC graph_slam::core graph_slam::frontend graph_slam::loopclosure)
set_global_target_properties(pipeline)
