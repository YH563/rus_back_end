#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>

namespace PointCloud {
    using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;  // 点云数据指针，为智能指针对象
    using MeshPtr = pcl::PolygonMeshPtr;  // 三角网格数据指针，为智能指针对象

    // 点云处理参数
    struct Parameters
    {
        double normal_radius;
        double gp3_radius;
        double gp3_mu;
        int gp3_max_nearest_neighbors;
        double gp3_max_surface_angle;
        bool gp3_normal_consistency;

        Parameters() : normal_radius(0.05)
            , gp3_radius(0.1)
            , gp3_mu(2.5)
            , gp3_max_nearest_neighbors(100)
            , gp3_max_surface_angle(M_PI / 4)
            , gp3_normal_consistency(false)
        {}
    };

    // 点云数据处理类
    class PointCloudPreprocess
    {
    public:
        PointCloudPreprocess() = default;
        PointCloudPreprocess(const Parameters& params) : params_(params) {};
        // 将点云数据转为三角网格
        MeshPtr Generate(const PointCloudPtr& cloud_ptr) const;
        // 获取当前处理参数
        Parameters GetParameters() const { return params_; }
        // 设置处理参数
        void SetParameters(const Parameters& params) { params_ = params; }
    private:
        Parameters params_;
    };
}