#pragma once

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rus_sim_interfaces/msg/detail/scan_path__struct.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include "rus_sim_planner/trajectory_planner.hpp"
#include "rus_sim_interfaces/action/scan_task.hpp"

namespace RusTrajectoryPlannerNode
{
    using TrajectoryPlanner = RusTrajectoryPlanner::TrajectoryPlanner;  // 轨迹规划器类
    using MsgMesh = shape_msgs::msg::Mesh;  // 三角网格消息类型
    using MsgMeshPtr = shape_msgs::msg::Mesh::SharedPtr;  // 三角网格数据指针
    using ScanTask = rus_sim_interfaces::action::ScanTask;  // 扫描任务动作定义

    // 轨迹规划节点类
    class TrajectoryPlannerNode : public rclcpp::Node
    {
    public:
        TrajectoryPlannerNode();

    private:
        // 接收网格数据的回调函数
        void on_mesh_data(const MsgMeshPtr& msg);

        // 决定受到目标后是否接收
        rclcpp_action::GoalResponse handle_scan_goal(const rclcpp_action::GoalUUID &uuid, 
            std::shared_ptr<const ScanTask::Goal> goal);

        // 客户端取消时调用
        rclcpp_action::CancelResponse handle_scan_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ScanTask>> goal_handle);

        // 处理扫描任务的执行
        void handle_scan_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ScanTask>> goal_handle);

        std::unique_ptr<TrajectoryPlanner> planner_;  // 轨迹规划器实例
        rclcpp::Subscription<MsgMesh>::SharedPtr mesh_subscription_;  // 三角网格数据订阅
        rclcpp_action::Server<ScanTask>::SharedPtr scan_action_server_;  // 扫描任务动作服务器
    };
}