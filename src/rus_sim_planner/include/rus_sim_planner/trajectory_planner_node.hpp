#pragma once

#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include "rus_sim_planner/trajectory_planner.hpp"

namespace TrajectoryPlannerNode
{
    // 轨迹规划节点类
    class TrajectoryPlannerNode : public rclcpp::Node
    {
    public:
        TrajectoryPlannerNode() : Node("trajectory_planner_node"), planner_()
        {
            RCLCPP_INFO(this->get_logger(), "轨迹规划节点已启动");
        }

    private:
        TrajectoryPlanner::TrajectoryPlanner planner_;  // 轨迹规划器实例
        rclcpp::Subscription<shape_msgs::msg::Mesh>::SharedPtr mesh_subscription_;  // 三角网格数据订阅
    };
}