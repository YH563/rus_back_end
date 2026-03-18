#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Core>
#include <optional>
#include "igl/heat_geodesics.h"
#include "igl/grad.h"

namespace TrajectoryPlanner {
    using Vector3 = Eigen::Vector3d;
    using Quaternion = Eigen::Quaterniond;
    using SE3 = Eigen::Isometry3d;  // 位姿矩阵
    using TrajectoryPoint = std::pair<SE3, double>;  // 位姿 + 时间戳
    using Trajectory = std::vector<TrajectoryPoint>;  // 轨迹由多个位姿点组成，每个点包含位姿和时间戳
    using PointCloud3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;  // 点云数据，N行3列，每行是一个点的(x,y,z)

    // 轨迹规划器
    class TrajectoryPlanner
    {
    public:
        TrajectoryPlanner() = default;
        ~TrajectoryPlanner() = default;
        // 初始化轨迹规划器
        bool Initialize(const SE3& start, const SE3& goal, const PointCloud3d& point_cloud, double total_time, double time_step = 0.1);
        // 生成轨迹
        bool GenerateTrajectory();
        // 获取生成的轨迹
        const Trajectory& GetTrajectory() const { return trajectory_; }
        // 按照时间戳获取位姿，若时间超出范围可返回空值
        std::optional<SE3> GetPoseAtTime(double time) const;

    private:
        SE3 start_pose_ = SE3::Identity();  // 起始位姿
        SE3 goal_pose_ = SE3::Identity();   // 目标位姿
        double total_time_ = 0.0;  // 总时间，单位为 s
        double time_step_ = 0.1;  // 时间步长
        PointCloud3d point_cloud_;  // 点云数据
        Trajectory trajectory_;  // 轨迹
    };

    // 轨迹规划节点类
    class TrajectoryPlannerNode : public rclcpp::Node
    {

    };
}