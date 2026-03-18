#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <random>
#include <cmath>

namespace TestPointCloud {
    // 测试用的点云生成器
    class PointCloudGenerator : public rclcpp::Node
    {
    public:
        PointCloudGenerator();
    private:
        void timer_callback();
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

}