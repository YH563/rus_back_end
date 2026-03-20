#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include "rus_sim_pointcloud/point_cloud_preprocess.hpp"

namespace PointCloudNode {
    using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;  // 点云数据指针，为智能指针对象
    using MeshPtr = pcl::PolygonMeshPtr;  // 三角网格数据指针，为智能指针对象

    // 点云处理节点类
    class PointCloudNode : public rclcpp::Node
    {
    public:
        PointCloudNode();
    private:
        // 订阅点云数据的回调函数
        void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // 将网格数据发布到ROS话题
        void publish_mesh(const MeshPtr& mesh);

        PointCloud::PointCloudPreprocess preprocess_;  // 点云处理器实例
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;  // 点云数据订阅
        rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr mesh_publisher_;  // 三角网格数据发布
    };
}