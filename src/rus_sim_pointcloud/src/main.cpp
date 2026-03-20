#include "rus_sim_pointcloud/point_cloud_node.hpp"
#include "test/point_cloud_generator.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto point_cloud_node = std::make_shared<PointCloudNode::PointCloudNode>();
    auto point_cloud_generator = std::make_shared<TestPointCloud::PointCloudGenerator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(point_cloud_node);
    executor.add_node(point_cloud_generator);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}