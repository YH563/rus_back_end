#include "rus_sim_planner/trajectory_planner.hpp"

namespace TrajectoryPlanner {
    bool TrajectoryPlanner::Initialize(const SE3 &start, const SE3 &goal, const MeshPtr& mesh, double total_time, double time_step)
    {
        if (total_time <= 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "总时间必须大于0");
            return false;
        }
        if (start.isApprox(goal)) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "起始位姿和目标位姿不能相同");
            return false;
        }
        start_pose_ = start;
        goal_pose_ = goal;
        if (!pclmesh_to_eigen(mesh)) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "网格数据转换失败");
            return false;
        }
        total_time_ = total_time;
        time_step_ = time_step;
        return true;
    }

    bool TrajectoryPlanner::pclmesh_to_eigen(const MeshPtr& mesh)
    {
        if (mesh->polygons.empty() || mesh->cloud.data.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "输入网格为空");
            return false;
        }
        
    }

    std::optional<SE3> TrajectoryPlanner::GetPoseAtTime(double time) const
    {
        // 检查时间戳是否在轨迹范围内
        if (time < 0.0 || time > total_time_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "时间戳超出范围: %f", time);
            return std::nullopt;
        }
        else 
        {
            // 在轨迹中查找对应时间戳的位姿
            for (const auto& point : trajectory_)
            {
                // 误差小于1e-6认为是同一时间戳
                if (std::abs(point.second - time) < 1e-6) {
                    return point.first;  // 返回对应时间戳的位姿
                }
                // 对于未找到的时间戳，进行插值计算
                if (time > point.second && time < point.second + time_step_) {
                    // 位置进行线性插值
                    double ratio = (time - point.second) / time_step_;
                    Vector3 p_interp = point.first.translation() + ratio * (goal_pose_.translation() - point.first.translation());
                    // 姿态进行球面线性插值（Slerp）
                    Quaternion q_pre(point.first.rotation());
                    Quaternion q_goal(goal_pose_.rotation());
                    Quaternion q_interp = q_pre.slerp(ratio, q_goal);

                    SE3 pose_interp = SE3::Identity();
                    pose_interp.translate(p_interp);
                    pose_interp.rotate(q_interp);
                    return pose_interp;
                }
                else
                    continue;
            }
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "未找到对应时间戳的位姿: %f", time);
            return std::nullopt;
        }
    }

    bool TrajectoryPlanner::GenerateTrajectory()
    {
        if (total_time_ <= 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "总时间必须大于0");
            return false;
        }
        if (start_pose_.isApprox(goal_pose_)) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "起始位姿和目标位姿不能相同");
            return false;
        }
        if (mesh_data_.first.rows() == 0 || mesh_data_.second.rows() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "三角网格数据不能为空");
            return false;
        }

        
        // TODO
        return true;
    }
}