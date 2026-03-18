#include "rus_sim_planning/trajectory_planner.hpp"

namespace TrajectoryPlanner {
    bool TrajectoryPlanner::Initialize(const SE3 &start, const SE3 &goal, const PointCloud3d &point_cloud, double total_time, double time_step)
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
        point_cloud_ = point_cloud;
        goal_pose_ = goal;
        total_time_ = total_time;
        time_step_ = time_step;
        return true;
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

    }
}