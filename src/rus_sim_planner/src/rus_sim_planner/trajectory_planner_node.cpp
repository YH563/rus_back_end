#include "rus_sim_planner/trajectory_planner_node.hpp"

namespace RusTrajectoryPlannerNode
{
    TrajectoryPlannerNode::TrajectoryPlannerNode() : Node("trajectory_planner_node")
    {
        // 创建轨迹规划器实例
        planner_ = std::make_unique<RusTrajectoryPlanner::TrajectoryPlanner>();
        mesh_subscription_ = this->create_subscription<shape_msgs::msg::Mesh>(
            "/mesh_data", 10, 
            std::bind(&TrajectoryPlannerNode::on_mesh_data, this, std::placeholders::_1)
        );
        // 创建扫描任务动作服务器
        scan_action_server_ = rclcpp_action::create_server<ScanTask>(
            this,
            "scan_task",
            std::bind(&TrajectoryPlannerNode::handle_scan_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectoryPlannerNode::handle_scan_cancel, this, std::placeholders::_1),
            std::bind(&TrajectoryPlannerNode::handle_scan_accepted, this, std::placeholders::_1)
        );
    }

    void TrajectoryPlannerNode::on_mesh_data(const MsgMeshPtr& msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收到环境网格数据，准备进行轨迹规划");
        bool is_initialized = planner_->Initialize(msg, 10.0, 0.1);  // 初始化轨迹规划器
        if (!is_initialized)
        {
            RCLCPP_ERROR(this->get_logger(), "轨迹规划器初始化失败！");
            return;
        }
    }

    rclcpp_action::GoalResponse TrajectoryPlannerNode::handle_scan_goal(const rclcpp_action::GoalUUID &uuid, 
        std::shared_ptr<const ScanTask::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "收到新的扫描任务请求");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;  // 接受并执行
    }

    rclcpp_action::CancelResponse TrajectoryPlannerNode::handle_scan_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ScanTask>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "扫描任务取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;  // 接受取消请求
    }

    void TrajectoryPlannerNode::handle_scan_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ScanTask>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "扫描任务已接受，正在执行...");
        // 这里可以添加执行扫描任务的逻辑，例如调用轨迹规划器生成轨迹并控制机器人执行
    }   
}