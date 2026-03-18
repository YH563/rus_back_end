// 正式用头文件
#include "rus_sim_planning/trajectory_planner.hpp"

// 测试用头文件
#include "point_cloud_generator.hpp"

int main()
{
    // 测试代码
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<TestPointCloud::PointCloudGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    // 测试代码


    return 0;
    
}