#include "rus_sim_pointcloud/point_cloud_preprocess.hpp"

namespace PointCloud {
    MeshPtr PointCloudPreprocess::Generate(const PointCloudPtr& cloud_ptr) const
    {
        if (cloud_ptr->empty()) {
            RCLCPP_WARN(rclcpp::get_logger("PointCloudPreprocess"), "输入点云为空，无法生成三角网格");
            return nullptr;
        }

        // 法线估计
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        normal_estimation.setInputCloud(cloud_ptr);
        normal_estimation.setRadiusSearch(params_.normal_radius);
        normal_estimation.compute(*normals);

        if (normals->empty()) {
            RCLCPP_WARN(rclcpp::get_logger("PointCloudPreprocess"), "法线估计失败，无法生成三角网格");
            return nullptr;
        }

        // 合并点云和法线
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*cloud_ptr, *normals, *cloud_with_normals);

        // 贪婪投影三角化
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(cloud_with_normals);

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        gp3.setSearchRadius(params_.gp3_radius);
        gp3.setMu(params_.gp3_mu);
        gp3.setMaximumNearestNeighbors(params_.gp3_max_nearest_neighbors);
        gp3.setMaximumSurfaceAngle(params_.gp3_max_surface_angle);
        gp3.setNormalConsistency(params_.gp3_normal_consistency);
        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree);
        pcl::PolygonMeshPtr mesh_ptr(new pcl::PolygonMesh);
        gp3.reconstruct(*mesh_ptr);
        return mesh_ptr;
    }
}