#include "point_cloud_generator.hpp"

namespace TestPointCloud {
    PointCloudGenerator::PointCloudGenerator() : Node("point_cloud_generator")
    {
        // 1. 声明参数
        this->declare_parameter<std::string>("topic_name", "/simulated_pointcloud");
        this->declare_parameter<double>("publish_rate", 1.0); // Hz
        this->declare_parameter<std::string>("frame_id", "world");

        // 2. 创建发布者
        std::string topic_name = this->get_parameter("topic_name").as_string();
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);

        // 3. 创建定时器
        double rate = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&PointCloudGenerator::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "点云生成器已启动，将在话题 [%s] 上发布数据", topic_name.c_str());
    }

    void PointCloudGenerator::timer_callback()
    {
        // 生成模拟点云数据：在一个平面上生成随机点
        constexpr size_t num_points = 1000;
        constexpr float plane_size = 2.0f;  // 平面范围 [-1, 1] 米

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = this->get_parameter("frame_id").as_string();

        // 设置点云字段：x, y, z, intensity
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(num_points);

        // 迭代器填充数据
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");

        // 随机数生成
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> pos_dis(-plane_size / 2, plane_size / 2);
        std::uniform_real_distribution<> intensity_dis(0.0, 1.0);

        // 生成点云（带一些高度变化，模拟真实表面）
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            float x = static_cast<float>(pos_dis(gen));
            float y = static_cast<float>(pos_dis(gen));
            // 添加一些正弦波高度变化，模拟曲面
            float z = 0.1f * std::sin(x * 3.14159f) * std::cos(y * 3.14159f);
            
            *iter_x = x;
            *iter_y = y;
            *iter_z = z;
            *iter_intensity = static_cast<float>(intensity_dis(gen));
        }

        // 发布点云
        publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布了包含 %zu 个点的点云", num_points);
    }

}
