#include "rus_sim_pointcloud/point_cloud_node.hpp"

namespace RusPointCloudNode {
    PointCloudNode::PointCloudNode() : Node("point_cloud_node")
    {
        // 声明参数
        this->declare_parameter<double>("normal_radius", 0.05);   // 法线估计半径
        this->declare_parameter<double>("gp3_radius", 0.1);      // 贪婪投影搜索半径
        this->declare_parameter<double>("gp3_mu", 2.5);           // 最大最近邻距离因子
        this->declare_parameter<int>("gp3_max_nearest_neighbors", 100);
        this->declare_parameter<double>("gp3_max_surface_angle", M_PI/4); // 45 degrees
        this->declare_parameter<bool>("gp3_normal_consistency", false);

        // 订阅点云数据
        point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/simulated_pointcloud", 10,
            std::bind(&PointCloudNode::on_point_cloud, this, std::placeholders::_1)
        );

        // 发布网络数据
        mesh_publisher_ = this->create_publisher<shape_msgs::msg::Mesh>("/mesh_data", 10);

        RCLCPP_INFO(this->get_logger(), "点云处理节点已启动");
    }

    void PointCloudNode::on_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        PointCloudPtr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_ptr); // 将ROS消息转换为PCL点云
        if (cloud_ptr->empty()) {
            RCLCPP_WARN(this->get_logger(), "接收到的点云数据为空");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "接收到点云数据，点数: %zu", cloud_ptr->size());
        }

        
        MeshPtr mesh_ptr = preprocess_.Generate(cloud_ptr); // 生成三角网格
        if (mesh_ptr != nullptr) {
            publish_mesh(mesh_ptr);
            RCLCPP_INFO(this->get_logger(), "已发布三角网格");
        } else {
            RCLCPP_WARN(this->get_logger(), "三角网格生成失败，未发布");
            return;
        }
        
    }

    void PointCloudNode::publish_mesh(const MeshPtr &mesh_ptr)
    {
        shape_msgs::msg::Mesh mesh_msg;
        
        // 转换顶点数据
        PointCloudPtr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(mesh_ptr->cloud, *cloud_ptr);

        for (const auto& point : cloud_ptr->points) {
            geometry_msgs::msg::Point vertex;
            vertex.x = point.x;
            vertex.y = point.y;
            vertex.z = point.z;
            mesh_msg.vertices.push_back(vertex);
        }

        for (const auto& polygon : mesh_ptr->polygons) {
            // 只处理三角形面片（PCL Mesh可能包含非三角形，这里做过滤）
            if (polygon.vertices.size() == 3) {
                shape_msgs::msg::MeshTriangle triangle;
                triangle.vertex_indices[0] = polygon.vertices[0];
                triangle.vertex_indices[1] = polygon.vertices[1];
                triangle.vertex_indices[2] = polygon.vertices[2];
                mesh_msg.triangles.push_back(triangle);
            }
        }
        mesh_publisher_->publish(mesh_msg);
        RCLCPP_INFO(this->get_logger(), "三角网格已发布，顶点数量: %zu, 面片数量: %zu", 
        mesh_msg.vertices.size(), mesh_msg.triangles.size());
    }
}