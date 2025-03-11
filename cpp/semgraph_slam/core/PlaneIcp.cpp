// This file is covered by the LICENSE file in the root of this project.
// contact: Neng Wang, <neng.wang@hotmail.com>



#include <iostream>
#include "PlaneIcp.hpp"

namespace graph_slam{

PlaneIcp::PlaneIcp(double map_voxel_size)
{
    map_voxel_size_ = map_voxel_size;
}

PlaneIcp::~PlaneIcp()
{

}

/*
    ICP refine for loop transformaion
*/
Eigen::Matrix4d PlaneIcp::getTransPlaneIcp(const V3d_i &cloud_a, const V3d_i &cloud_b, const Eigen::Matrix4d &init_trans){

    auto plane_cloud_a = extractPlane(cloud_a,map_voxel_size_);
    auto plane_cloud_b = extractPlane(cloud_b,map_voxel_size_);

    auto trans_planeicp = PlaneAlign(plane_cloud_a,plane_cloud_b,init_trans,map_voxel_size_);

    return trans_planeicp;
}


std::vector<Eigen::Vector6d>PlaneIcp::extractPlane(const V3d_i &cloud, double voxel_size){

    std::vector<Eigen::Vector6d> plane_cloud;

    std::vector<Eigen::Vector4d> points_4d(cloud.first.size());

    tbb::parallel_for(size_t(0),cloud.first.size(), [&](size_t i){
        points_4d[i].head<3>() =  cloud.first[i];
        points_4d[i](3) = cloud.second[i];
    });

    tsl::robin_map<Eigen::Vector3i,VoxelBlock,VoxelHash> voxel_map;
    std::for_each(points_4d.cbegin(), points_4d.cend(), [&](const Eigen::Vector4d &point) {
        auto voxel = Eigen::Vector3i((point.head<3>()/voxel_size).template cast<int>());
        auto search = voxel_map.find(voxel);
        if (search != voxel_map.end()) {
            auto &voxel_block = search.value();
            voxel_block.voxel_points_.push_back(point);
        } else {
            voxel_map.insert({voxel, VoxelBlock{{point}}});
        }
    });


    // extract plane -> for each voxel 
    for(auto&[voxel,voxel_block]:voxel_map){
        (void)voxel;
        auto voxel_points = voxel_block.voxel_points_;

        Eigen::Vector6d plane;
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();

        if((int)voxel_points.size()<min_point_voxel_) continue;

        for(auto &point:voxel_points){
            Eigen::Vector3d pointv3 = point.head<3>();
            center += pointv3;
            cov    += pointv3 * pointv3.transpose();
        }
        center /= (int) voxel_points.size();
        cov    /= (int) voxel_points.size();
        cov    -= center * center.transpose();

        //decomposing
        Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3d::Index evalsMin;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);

        // plane check
        if (evalsReal(evalsMin) > plane_thre_) continue;

        plane[0] = center.x();
        plane[1] = center.y();
        plane[2] = center.z();
        plane[3] = evecs.real()(0, evalsMin);
        plane[4]= evecs.real()(1, evalsMin);
        plane[5] = evecs.real()(2, evalsMin);
        plane_cloud.emplace_back(plane);

    }
    
    return plane_cloud;
}

Eigen::Matrix4d PlaneIcp::PlaneAlign(const std::vector<Eigen::Vector6d> &cloud_a,
                                        const std::vector<Eigen::Vector6d> &cloud_b,
                                           const Eigen::MatrixX4d &init_trans,double voxel_size)
{
    tsl::robin_map<Eigen::Vector3i,VoxelBlock6d,VoxelHash> voxel_map;
    std::for_each(cloud_b.cbegin(), cloud_b.cend(), [&](const Eigen::Vector6d &point) {
        auto voxel = Eigen::Vector3i((point.head<3>()/voxel_size).template cast<int>());
        auto search = voxel_map.find(voxel);
        if (search != voxel_map.end()) {
            auto &voxel_block = search.value();
            voxel_block.voxel_points_.push_back(point);
        } else {
            voxel_map.insert({voxel, VoxelBlock6d{{point}}});
        }
    });

    // trans
    Eigen::Isometry3d init_trans_o3d = Eigen::Isometry3d::Identity();
    init_trans_o3d.matrix() = init_trans;
    // std::transform(cloud_a.begin(),cloud_a.end(),cloud_a.begin(),[&](Eigen::Vector4d &point){
    //             point.head<3>() = init_trans_o3d * point.head<3>();});

    // cere 
    ceres::Manifold *quaternion_manifold = new ceres::EigenQuaternionManifold;
    ceres::Problem problem;
    ceres::LossFunction *loss_function = nullptr;
    Eigen::Matrix3d rot = init_trans_o3d.linear();
    Eigen::Quaterniond q(rot); // init_trans_o3d.rotation()
    Eigen::Vector3d t = init_trans_o3d.translation();
    double para_q[4] = {q.x(), q.y(), q.z(), q.w()};
    double para_t[3] = {t(0), t(1), t(2)};
    problem.AddParameterBlock(para_q, 4, quaternion_manifold);
    problem.AddParameterBlock(para_t, 3);
    Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);
    for(size_t i=0; i < cloud_a.size(); i++){
        Eigen::Vector3d center_a = cloud_a[i].head<3>();
        Eigen::Vector3d normal_a = cloud_a[i].tail<3>();
        center_a = init_trans_o3d* center_a; // 转移中心点
        normal_a= init_trans_o3d.linear() * normal_a; // 转移法向量,

        auto kx = static_cast<int>(center_a[0] / voxel_size);
        auto ky = static_cast<int>(center_a[1] / voxel_size);
        auto kz = static_cast<int>(center_a[2] / voxel_size);

        std::vector<Eigen::Vector3i> voxels_negibor;
        voxels_negibor.reserve(27);
        for (int i = kx - 1; i < kx + 1 + 1; ++i) {
            for (int j = ky - 1; j < ky + 1 + 1; ++j) {
                for (int k = kz - 1; k < kz + 1 + 1; ++k) {
                    voxels_negibor.emplace_back(i, j, k);
                }
            }
        }
        std::vector<Eigen::Vector6d> neighbors;
        neighbors.reserve(27*5);
        std::for_each(voxels_negibor.cbegin(), voxels_negibor.cend(), [&](const auto &voxel) {
            auto search = voxel_map.find(voxel);
            if (search != voxel_map.end()) {
                const auto &points = search.value().voxel_points_;
                if (!points.empty()) {
                    for (const Eigen::Vector6d &point : points) {
                        neighbors.emplace_back(point);
                    }
                }
            }
        });

        if(neighbors.size()==0) continue; // no find neighbor

        Eigen::Vector6d closest_neighbor;
        double closest_distance2 = std::numeric_limits<double>::max();
        std::for_each(neighbors.cbegin(), neighbors.cend(), [&](const Eigen::Vector6d &neighbor) {
            double distance = (neighbor.head<3>() - center_a).squaredNorm();
            if (distance < closest_distance2) { 
                closest_neighbor = neighbor;
                closest_distance2 = distance;
            }
        });

        Eigen::Vector3d center_b = closest_neighbor.head<3>();
        Eigen::Vector3d normal_b = closest_neighbor.tail<3>();
        Eigen::Vector3d normal_inc = normal_a - normal_b;
        Eigen::Vector3d normal_add = normal_a + normal_b;
        double point_to_plane = fabs(normal_b.transpose() * (center_a - center_b));
        if ((normal_inc.norm() < normal_thr_ ||
           normal_add.norm() < normal_thr_) &&
          point_to_plane < point_to_plane_thr_ &&
          closest_distance2 < 3) {

            ceres::CostFunction *cost_function;
            cost_function = PlaneSolver::Create(cloud_a[i].head<3>(), cloud_a[i].tail<3>(), center_b, normal_b);
            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
          }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Eigen::Quaterniond q_opt(para_q[3], para_q[0], para_q[1], para_q[2]);
    Eigen::Isometry3d trans = Eigen::Isometry3d ::Identity();
    trans.linear() = q_opt.toRotationMatrix();
    trans.translation() = Eigen::Vector3d(t_last_curr(0), t_last_curr(1), t_last_curr(2));

    return trans.matrix();
}

} // namespace graph_slam