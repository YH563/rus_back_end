#include "rus_sim_planner/trajectory_planner.hpp"

namespace RusTrajectoryPlanner {
    bool TrajectoryPlanner::Initialize(const MsgMeshPtr& mesh, double total_time, double time_step)
    {
        if (total_time <= 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "总时间必须大于0");
            return false;
        }
        if (time_step <= 0.0) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "时间步长必须大于0");
            return false;
        }
        if (!pclmesh_to_eigen(mesh)) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "网格数据转换失败");
            return false;
        }
        total_time_ = total_time;
        time_step_ = time_step;
        is_initialized_ = true;
        return true;
    }

    bool TrajectoryPlanner::pclmesh_to_eigen(const MsgMeshPtr& mesh)
    {
        if (mesh->vertices.empty() || mesh->triangles.empty() || !mesh) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "输入网格为空");
            return false;
        }
        else 
        {
            size_t num_vertices = mesh->vertices.size();
            size_t num_faces = mesh->triangles.size();

            // 创建顶点和面矩阵
            mesh_data_.first.resize(num_vertices, 3);  // 顶点矩阵
            mesh_data_.second.resize(num_faces, 3);   // 面矩阵

            // 对顶点矩阵和面矩阵进行填充
            for (size_t i=0; i < num_vertices; ++i) {
                mesh_data_.first(i, 0) = mesh->vertices[i].x;
                mesh_data_.first(i, 1) = mesh->vertices[i].y;
                mesh_data_.first(i, 2) = mesh->vertices[i].z;
            }
            for (size_t i=0; i < num_faces; ++i) {
                if (mesh->triangles[i].vertex_indices.size() != 3) {
                    RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "非三角形面数据不支持");
                    return false;
                }
                mesh_data_.second(i, 0) = mesh->triangles[i].vertex_indices[0];
                mesh_data_.second(i, 1) = mesh->triangles[i].vertex_indices[1];
                mesh_data_.second(i, 2) = mesh->triangles[i].vertex_indices[2];
            }
            return true;
        }
        
    }

    std::optional<SE3> TrajectoryPlanner::GetPoseAtTime(double time) const
    {
        // 检查是否已初始化
        if (!is_initialized_) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "轨迹规划器未初始化");
            return std::nullopt;
        }
        // 检查轨迹是否已生成
        if (trajectory_.empty()) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "轨迹未生成");
            return std::nullopt;
        }
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

    bool TrajectoryPlanner::GenerateTrajectory(const SE3& start, const SE3& goal)
    {
        if (!is_initialized_) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "轨迹规划器未初始化");
            return false;
        }
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
        if (start.isApprox(goal)) {
            RCLCPP_ERROR(rclcpp::get_logger("TrajectoryPlanner"), "起始位姿和目标位姿不能相同");
            return false;
        }
        start_pose_ = start;
        goal_pose_ = goal;
        
        // TODO
        return true;
    }
}