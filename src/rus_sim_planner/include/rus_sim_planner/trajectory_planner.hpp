#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Core>
#include <optional>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/memory.h>

namespace TrajectoryPlanner {
    using Vector3 = Eigen::Vector3d;  // 3D向量
    using Quaternion = Eigen::Quaterniond;  // 四元数
    using SE3 = Eigen::Isometry3d;  // 位姿矩阵
    using TrajectoryPoint = std::pair<SE3, double>;  // 位姿 + 时间戳
    using Trajectory = std::vector<TrajectoryPoint>;  // 轨迹由多个位姿点组成，每个点包含位姿和时间戳
    using MeshPtr = pcl::PolygonMeshPtr;  // 三角网格数据指针，为智能指针对象
    using VertexMatrix = Eigen::MatrixXd;  // 网格顶点矩阵，每行表示一个顶点的坐标
    using FaceMatrix = Eigen::MatrixXi;  // 网格面矩阵，每行表示一个面的顶点索引

    // 轨迹规划器
    class TrajectoryPlanner
    {
    public:
        TrajectoryPlanner() = default;
        ~TrajectoryPlanner() = default;
        // 初始化轨迹规划器
        bool Initialize(const SE3& start, const SE3& goal, const MeshPtr& mesh, double total_time, double time_step = 0.1);
        // 生成轨迹
        bool GenerateTrajectory();
        // 获取生成的轨迹
        const Trajectory& GetTrajectory() const { return trajectory_; }
        // 按照时间戳获取位姿，若时间超出范围可返回空值
        std::optional<SE3> GetPoseAtTime(double time) const;

    private:
        bool pclmesh_to_eigen(const MeshPtr& mesh);  // 将PCL网格数据转换为Eigen矩阵，便于 libigl 进行处理计算
        std::pair<VertexMatrix, FaceMatrix> mesh_data_;  // 网格数据：顶点矩阵和面矩阵
        SE3 start_pose_ = SE3::Identity();  // 起始位姿
        SE3 goal_pose_ = SE3::Identity();   // 目标位姿
        double total_time_ = 0.0;  // 总时间，单位为 s
        double time_step_ = 0.1;  // 时间步长
        Trajectory trajectory_;  // 轨迹
    };
}